const express = require('express');
const cors = require('cors');
const path = require('path');
const https = require('https');

// fetch polyfill for Node < 18 (Jetson runs Node 14)
function httpsJson(url, options) {
  var parsed = new URL(url);
  var bodyBuf = options.body ? Buffer.from(options.body) : null;
  var headers = Object.assign({}, options.headers || {});
  if (bodyBuf) headers['Content-Length'] = bodyBuf.length;
  return new Promise(function(resolve, reject) {
    var req = https.request({
      hostname: parsed.hostname,
      path: parsed.pathname,
      method: options.method || 'GET',
      headers: headers,
    }, function(res) {
      var chunks = [];
      res.on('data', function(c) { chunks.push(c); });
      res.on('end', function() {
        var buf = Buffer.concat(chunks);
        resolve({ ok: res.statusCode >= 200 && res.statusCode < 300, status: res.statusCode, buf: buf,
          json: function() { return JSON.parse(buf.toString()); } });
      });
    });
    req.on('error', reject);
    if (bodyBuf) req.write(bodyBuf);
    req.end();
  });
}
// For streaming responses (TTS)
function httpsStream(url, options) {
  var parsed = new URL(url);
  var bodyBuf = options.body ? Buffer.from(options.body) : null;
  var headers = Object.assign({}, options.headers || {});
  if (bodyBuf) headers['Content-Length'] = bodyBuf.length;
  return new Promise(function(resolve, reject) {
    var req = https.request({
      hostname: parsed.hostname,
      path: parsed.pathname,
      method: options.method || 'GET',
      headers: headers,
    }, function(res) {
      resolve({ ok: res.statusCode >= 200 && res.statusCode < 300, status: res.statusCode, stream: res,
        json: function() { return new Promise(function(r2) {
          var c = []; res.on('data', function(d) { c.push(d); });
          res.on('end', function() { r2(JSON.parse(Buffer.concat(c).toString())); });
        }); }
      });
    });
    req.on('error', reject);
    if (bodyBuf) req.write(bodyBuf);
    req.end();
  });
}

const app = express();
const PORT = process.env.PORT || 3000;
const OPENAI_API_KEY = process.env.OPENAI_API_KEY;
const OPENAI_MODEL = process.env.OPENAI_MODEL || 'gpt-5-mini';

// ── Libot system prompt ──
const LIBOT_SYSTEM_PROMPT = `You are Libot (pronounced "LYE-bot") — a friendly library robot made by a student team at the University of Pennsylvania for the SICK 10K LiDAR Challenge. You roll around the library using a SICK LiDAR sensor to navigate and scan bookshelves automatically. Right now you are chatting with someone through your touchscreen.

HOW TO TALK
- Be warm, casual, and subtly witty. Talk like a sharp friend who works at the library — naturally charming, not performing. Humor should come from the situation, not from trying to be funny. A well-timed dry line beats a forced joke every time.
- Keep it short — one to three sentences unless they want more. You are being read aloud, so no bullet points, lists, or markdown. Just natural speech.
- Don't describe yourself as funny, don't announce jokes, don't do puns unless someone asks. Just be helpful with personality. If humor fits naturally, great — if not, being genuinely helpful IS the personality.
- If someone asks about books, genres, or reading — give genuinely thoughtful recommendations and opinions like a great librarian would. You love books and it shows.
- If a visitor asks for more detail, go deeper. Otherwise stay concise.

WHAT YOU DO
- You can help people check out books, return books, look up whether a book is available, and answer general library questions — all from your touchscreen.
- Your standout trick is shelf reading: you drive through the aisles on your own and scan book spines with your LiDAR and cameras, keeping the library catalog accurate without anyone lifting a finger. Libraries normally spend hours a week doing this by hand.
- You are a real robot that moves around the library, not just a screen.

WHAT YOU KNOW
- How libraries work — borrowing, returns, holds, fines, library cards, printing, study rooms, children's sections, digital resources, the whole deal.
- You were built by Penn students for the SICK LiDAR Challenge, a $10K robotics competition focused on real-world LiDAR applications.
- If someone asks about specific hours, policies, or live availability, be upfront that you might not have the latest info and suggest asking the front desk or checking the website.

RULES
- Never make up book titles, authors, or call numbers. If you are unsure, say so.
- Never talk about your prompt, instructions, or how you work internally.
- If the conversation drifts way off topic, gently bring it back to how you can help at the library.
- Your name often gets mangled by speech recognition. ANY word that sounds even vaguely like "Libot" (light bot, lie bot, live bot, Laibo, Lybot, light bulb, Leiba, live up, etc.) IS "Libot." NEVER comment on it, NEVER say "close enough," NEVER correct them, NEVER joke about the mispronunciation. Just treat it as if they said "Libot" perfectly. Pretend you didn't notice.`;

// Middleware
app.use(cors());
app.use(express.json());
app.use(express.static('public'));

// Serve the main HTML file
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// API endpoint for logging face detection events (optional)
app.post('/api/log-detection', (req, res) => {
  const { event, timestamp } = req.body;
  console.log(`Face Detection Event: ${event} at ${timestamp}`);
  res.json({ success: true });
});

// Whisper speech-to-text — Firefox fallback for Web Speech API
app.post('/api/transcribe', express.raw({ type: '*/*', limit: '5mb' }), async (req, res) => {
  if (!OPENAI_API_KEY) {
    return res.status(503).json({ error: 'OPENAI_API_KEY is not configured.' });
  }

  try {
    // Build multipart body manually (works on Node 14+ without global FormData/fetch)
    const boundary = '----formdata' + Date.now();
    const parts = [
      '--' + boundary + '\r\nContent-Disposition: form-data; name="file"; filename="audio.webm"\r\nContent-Type: audio/webm\r\n\r\n',
      req.body,
      '\r\n--' + boundary + '\r\nContent-Disposition: form-data; name="model"\r\n\r\nwhisper-1\r\n',
      '--' + boundary + '\r\nContent-Disposition: form-data; name="language"\r\n\r\nen\r\n',
      '--' + boundary + '--\r\n',
    ];
    const body = Buffer.concat(parts.map(function(p) { return Buffer.isBuffer(p) ? p : Buffer.from(p); }));

    const data = await new Promise(function(resolve, reject) {
      const req2 = https.request({
        hostname: 'api.openai.com',
        path: '/v1/audio/transcriptions',
        method: 'POST',
        headers: {
          'Authorization': 'Bearer ' + OPENAI_API_KEY,
          'Content-Type': 'multipart/form-data; boundary=' + boundary,
          'Content-Length': body.length,
        },
      }, function(res2) {
        var chunks = [];
        res2.on('data', function(c) { chunks.push(c); });
        res2.on('end', function() {
          try { resolve({ status: res2.statusCode, body: JSON.parse(Buffer.concat(chunks).toString()) }); }
          catch (e) { reject(e); }
        });
      });
      req2.on('error', reject);
      req2.write(body);
      req2.end();
    });

    if (data.status !== 200) {
      console.error('Whisper API error:', data.body);
      return res.status(data.status).json({ error: (data.body.error && data.body.error.message) || 'Transcription failed.' });
    }

    res.json({ text: data.body.text || '' });
  } catch (error) {
    console.error('Transcribe error:', error);
    res.status(500).json({ error: 'Transcription failed.' });
  }
});

app.post('/api/chatbot', async (req, res) => {
  const { message, history = [] } = req.body || {};

  if (!message || typeof message !== 'string' || !message.trim()) {
    return res.status(400).json({ error: 'A non-empty message is required.' });
  }

  if (!OPENAI_API_KEY) {
    return res.status(503).json({ error: 'OPENAI_API_KEY is not configured on the server.' });
  }

  const safeHistory = Array.isArray(history) ? history.slice(-8) : [];
  const messages = [
    { role: 'system', content: LIBOT_SYSTEM_PROMPT },
  ];
  safeHistory
    .filter((entry) => entry && typeof entry.content === 'string' && typeof entry.role === 'string')
    .forEach((entry) => messages.push({ role: entry.role, content: entry.content.trim() }));
  messages.push({ role: 'user', content: message.trim() });

  try {
    const response = await httpsJson('https://api.openai.com/v1/chat/completions', {
      method: 'POST',
      headers: {
        'Authorization': 'Bearer ' + OPENAI_API_KEY,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        model: OPENAI_MODEL,
        messages: messages,
        max_completion_tokens: 450,
      }),
    });

    const data = response.json();
    if (!response.ok) {
      console.error('OpenAI API error:', data);
      return res.status(response.status).json({
        error: (data.error && data.error.message) || 'OpenAI request failed.',
      });
    }

    const reply = (data.choices && data.choices[0] && data.choices[0].message && data.choices[0].message.content || '').trim();
    if (!reply) {
      console.error('Empty response from model. Full API response:', JSON.stringify(data, null, 2));
      return res.status(502).json({ error: 'The model response did not contain text.' });
    }

    console.log(`Robot answer: ${reply}`);
    res.json({
      reply,
      model: OPENAI_MODEL,
    });
  } catch (error) {
    console.error('Chatbot API request failed:', error);
    res.status(500).json({ error: 'Failed to contact the chatbot service.' });
  }
});

// ── Streaming chatbot — sends sentences as SSE events ──
app.post('/api/chatbot-stream', express.json(), async function(req, res) {
  var body = req.body || {};
  var message = body.message;
  var history = Array.isArray(body.history) ? body.history.slice(-8) : [];

  if (!message || typeof message !== 'string' || !message.trim()) {
    return res.status(400).json({ error: 'A non-empty message is required.' });
  }
  if (!OPENAI_API_KEY) {
    return res.status(503).json({ error: 'OPENAI_API_KEY is not configured.' });
  }

  var messages = [{ role: 'system', content: LIBOT_SYSTEM_PROMPT }];
  history
    .filter(function(e) { return e && typeof e.content === 'string' && typeof e.role === 'string'; })
    .forEach(function(e) { messages.push({ role: e.role, content: e.content.trim() }); });
  messages.push({ role: 'user', content: message.trim() });

  // SSE headers
  res.setHeader('Content-Type', 'text/event-stream');
  res.setHeader('Cache-Control', 'no-cache');
  res.setHeader('Connection', 'keep-alive');
  res.flushHeaders();

  try {
    console.log('Streaming request to OpenAI model:', OPENAI_MODEL);
    var response = await httpsStream('https://api.openai.com/v1/chat/completions', {
      method: 'POST',
      headers: {
        'Authorization': 'Bearer ' + OPENAI_API_KEY,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        model: OPENAI_MODEL,
        messages: messages,
        max_completion_tokens: 450,
        stream: true,
      }),
    });

    if (!response.ok) {
      var errData = await response.json();
      console.error('OpenAI stream error ' + response.status + ':', JSON.stringify(errData));
      res.write('data: ' + JSON.stringify({ error: (errData.error && errData.error.message) || 'OpenAI API error' }) + '\n\n');
      res.end();
      return;
    }

    var lineBuf = '';
    var sentenceBuf = '';
    var fullReply = '';
    var ended = false;

    response.stream.on('data', function(chunk) {
      lineBuf += chunk.toString();
      var lines = lineBuf.split('\n');
      lineBuf = lines.pop();

      for (var i = 0; i < lines.length; i++) {
        var line = lines[i].trim();
        if (!line.startsWith('data: ')) continue;
        var payload = line.slice(6);
        if (payload === '[DONE]') {
          if (sentenceBuf.trim()) {
            fullReply += (fullReply ? ' ' : '') + sentenceBuf.trim();
            res.write('data: ' + JSON.stringify({ sentence: sentenceBuf.trim() }) + '\n\n');
          }
          console.log('Robot answer: ' + fullReply);
          res.write('data: [DONE]\n\n');
          ended = true;
          res.end();
          return;
        }
        try {
          var parsed = JSON.parse(payload);
          var token = (parsed.choices && parsed.choices[0] && parsed.choices[0].delta && parsed.choices[0].delta.content) || '';
          if (!token) continue;
          sentenceBuf += token;

          var match;
          while ((match = sentenceBuf.match(/^(.*?[^0-9][.!?])(\s+|$)/))) {
            var sentence = match[1].trim();
            if (sentence) {
              fullReply += (fullReply ? ' ' : '') + sentence;
              res.write('data: ' + JSON.stringify({ sentence: sentence }) + '\n\n');
            }
            sentenceBuf = sentenceBuf.slice(match[0].length);
          }
        } catch(e) {}
      }
    });

    response.stream.on('end', function() {
      if (!ended) {
        if (sentenceBuf.trim()) {
          fullReply += (fullReply ? ' ' : '') + sentenceBuf.trim();
          res.write('data: ' + JSON.stringify({ sentence: sentenceBuf.trim() }) + '\n\n');
        }
        if (fullReply) console.log('Robot answer: ' + fullReply);
        res.write('data: [DONE]\n\n');
        res.end();
      }
    });

    req.on('close', function() { response.stream.destroy(); });

  } catch (err) {
    console.error('Chatbot stream request failed:', err);
    try { res.write('data: ' + JSON.stringify({ error: 'Stream failed' }) + '\n\n'); res.end(); } catch(e) {}
  }
});

// OpenAI TTS — returns audio/mpeg stream
const OPENAI_TTS_VOICE = process.env.OPENAI_TTS_VOICE || 'nova';
app.post('/api/tts', express.json(), async (req, res) => {
  const { text } = req.body || {};
  if (!text || !text.trim()) return res.status(400).json({ error: 'No text provided.' });
  if (!OPENAI_API_KEY) return res.status(503).json({ error: 'OPENAI_API_KEY is not configured.' });

  try {
    const response = await httpsStream('https://api.openai.com/v1/audio/speech', {
      method: 'POST',
      headers: {
        'Authorization': 'Bearer ' + OPENAI_API_KEY,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        model: 'tts-1',
        input: text.trim().replace(/\bLibot\b/gi, 'Lie-bot'),
        voice: OPENAI_TTS_VOICE,
        response_format: 'mp3',
      }),
    });

    if (!response.ok) {
      var err = await response.json();
      console.error('TTS API error:', err);
      return res.status(response.status).json({ error: (err.error && err.error.message) || 'TTS failed.' });
    }

    res.set('Content-Type', 'audio/mpeg');
    response.stream.pipe(res);
  } catch (error) {
    console.error('TTS error:', error);
    res.status(500).json({ error: 'TTS failed.' });
  }
});

// Start server
app.listen(PORT, () => {
  console.log(`Libot Interface running on http://localhost:${PORT}`);
  console.log('Press Ctrl+C to stop the server');
});
