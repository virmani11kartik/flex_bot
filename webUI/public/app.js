/**
 * Library Robot Interface - Main Application
 * Integrates face detection with UI screens
 */
 
// ============================================================================
// CONFIGURATION - Adjust these values to customize behavior
// ============================================================================
 
// Set to false to disable camera + face detection (saves significant CPU on Jetson)
const ENABLE_FACE_DETECTION = false;

// Inactivity Timeouts (in milliseconds)
const INACTIVITY_TIMEOUT = 30000; // 30 seconds - Return to home after no clicks/touches
const NO_FACE_TIMEOUT = 15000;    // 15 seconds - Return to home when face not detected
 
// Face Detection Settings
const FACE_DETECTION_CONFIG = {
  minDetectionConfidence: 0.8,    // Minimum confidence for face detection (0-1)
  requiredGazeDuration: 2000,     // Time in ms a face must remain visible before engaging
  faceSizeThreshold: 0.12,        // Minimum face size to prevent distant detections (0-1)
  detectionInterval: 100,         // Time between detection checks in ms
  obstructionDiffThreshold: 28,   // Average frame difference to treat as close obstruction
  obstructionCoverageThreshold: 0.35, // Portion of changed pixels that counts as obstruction
  obstructionDarknessThreshold: 55,   // Low brightness threshold for covered camera detection
  obstructionPixelThreshold: 35,      // Per-pixel grayscale delta for changed coverage
  baselineLearningRate: 0.08,         // Speed to adapt the background reference frame
  analysisWidth: 48,
  analysisHeight: 36,
};

const ROS_BRIDGE_URL = 'http://localhost:8080';
const STAND_BY_POSE = {
  x: 0.0,
  y: 0.0,
  theta: 0.0,
  frame_id: 'map',
};

const HOME_POSE = {
  x: 0.0,
  y: 0.0,
  theta: 0.0,
  frame_id: 'map',
};
 
// ============================================================================
 
// UI State Management
let currentScreen = 'defaultScreen';
let debugMode = false; // Start with debug mode off
let previousScreenBeforeAdmin = 'defaultScreen';
 
// Inactivity tracking
let inactivityTimeout = null;
let noFaceTimeout = null;
 
// Face Detection
let faceDetector = null;
let video = null;
let overlayCanvas = null;
let overlayContext = null;

// Voice Assistant
let speechRecognition = null;
let speechRecognitionSupported = false;
let isVoiceListening = false;
let isVoiceSpeaking = false;
let isVoiceProcessing = false; // true while waiting for API response or TTS
let pendingVoiceResponse = '';
let manualVoiceStopRequested = false;
let voiceSessionActive = false;
let voiceRestartTimeout = null;
let lastVoiceRecognitionError = '';
let _lastTtsEndTime = 0;
let voiceConversationHistory = [];
let voiceHistoryTimeout = null;
const VOICE_HISTORY_TTL = 60000; // 1 minute of inactivity before clearing context

function resetVoiceHistoryTimeout() {
  if (voiceHistoryTimeout) clearTimeout(voiceHistoryTimeout);
  voiceHistoryTimeout = setTimeout(function() {
    voiceConversationHistory = [];
    console.log('Voice context cleared after 1 min inactivity');
    // Return to default screen if voice assistant is off
    if (!voiceSessionActive && !isVoiceSpeaking && !isVoiceProcessing) {
      stopVoiceAssistant(true);
      returnToDefault();
    }
  }, VOICE_HISTORY_TTL);
}

function clearVoiceHistory() {
  if (voiceHistoryTimeout) { clearTimeout(voiceHistoryTimeout); voiceHistoryTimeout = null; }
  voiceConversationHistory = [];
}
 
// Book Returns
let bookReturnsManager = null;
let selfCheckoutManager = null;
 
// DOM Elements
const elements = {
  video: null,
  overlay: null,
  robotFace: null,
  robotStatusText: null,
  speakButton: null,
  voiceAssistantMessage: null,
  voiceTranscriptText: null,
  detectionPanel: null,
  cameraStatus: null,
  modelStatus: null,
  faceStatus: null,
  gazeStatus: null,
  progressFill: null,
  progressText: null,
};
 
/**
 * Initialize the application
 */
async function init() {
  console.log('Initializing Library Robot Interface...');
  
  // Get DOM elements
  elements.video = document.getElementById('video');
  elements.overlay = document.getElementById('overlay');
  elements.robotFace = document.querySelector('#defaultScreen .robot-face');
  elements.robotStatusText = document.getElementById('robotStatusText');
  elements.speakButton = document.getElementById('speakButton');
  elements.voiceAssistantMessage = document.getElementById('voiceAssistantMessage');
  elements.voiceTranscriptText = document.getElementById('voiceTranscriptText');
  
  video = elements.video;
  overlayCanvas = elements.overlay;
  overlayContext = overlayCanvas.getContext('2d');
  
  // Initialize camera + face detection (skip if disabled for performance)
  if (ENABLE_FACE_DETECTION) {
    await setupCamera();
    await initializeFaceDetection();
  } else {
    console.log('Face detection disabled (ENABLE_FACE_DETECTION = false)');
  }
  
  // Initialize book returns manager
  initializeBookReturns();

  // Initialize self checkout manager
  initializeSelfCheckout();

  // Initialize voice assistant
  initializeVoiceAssistant();
  
  // Load books database
  await loadBooksDatabase();
}
 
/**
 * Setup camera access
 */
async function setupCamera() {
  try {
    console.log('Requesting camera access...');
    
    const stream = await navigator.mediaDevices.getUserMedia({
      video: {
        facingMode: 'user',
        width: { ideal: 640 },
        height: { ideal: 480 }
      },
      audio: false
    });
    
    video.srcObject = stream;
    
    // Wait for video to be ready
    await new Promise((resolve) => {
      video.onloadedmetadata = () => {
        video.play();
        resolve();
      };
    });
    
    // Set overlay canvas size to match video
    overlayCanvas.width = video.videoWidth;
    overlayCanvas.height = video.videoHeight;
    
    console.log('Camera initialized successfully');
    
    return true;
  } catch (error) {
    console.error('Camera initialization error:', error);
    alert('Camera access is required for face detection. Please allow camera access and reload.');
    return false;
  }
}
 
/**
 * Initialize face detection with BlazeFace
 */
async function initializeFaceDetection() {
  try {
    console.log('Loading BlazeFace model...');
    
    // Load BlazeFace model
    const model = await blazeface.load();
    
    console.log('BlazeFace model loaded successfully');
    
    // Create face detector instance
    faceDetector = new FaceDetectorManager(model, video, FACE_DETECTION_CONFIG);
    
    // Set up callbacks
    faceDetector.onGazeStart = handleGazeStart;
    faceDetector.onGazeEnd = handleGazeEnd;
    faceDetector.onEngaged = handleUserEngaged;
    
    // Start detection
    faceDetector.start();
    
    console.log('Face detection started');
  } catch (error) {
    console.error('Face detection initialization error:', error);
  }
}
 
/**
 * Face Detector Manager Class
 */
class FaceDetectorManager {
  constructor(model, videoElement, config = {}) {
    this.model = model;
    this.video = videoElement;
    this.isRunning = false;
    this.hasTriggeredWelcome = false;
    
    // Configuration
    this.config = {
      minDetectionConfidence: config.minDetectionConfidence || 0.8,
      requiredGazeDuration: config.requiredGazeDuration || 2000,
      faceSizeThreshold: config.faceSizeThreshold || 0.15,
      detectionInterval: config.detectionInterval || 100,
      obstructionDiffThreshold: config.obstructionDiffThreshold || 28,
      obstructionCoverageThreshold: config.obstructionCoverageThreshold || 0.35,
      obstructionDarknessThreshold: config.obstructionDarknessThreshold || 55,
      obstructionPixelThreshold: config.obstructionPixelThreshold || 35,
      baselineLearningRate: config.baselineLearningRate || 0.08,
      analysisWidth: config.analysisWidth || 48,
      analysisHeight: config.analysisHeight || 36,
    };

    this.analysisCanvas = document.createElement('canvas');
    this.analysisCanvas.width = this.config.analysisWidth;
    this.analysisCanvas.height = this.config.analysisHeight;
    this.analysisContext = this.analysisCanvas.getContext('2d', { willReadFrequently: true });
    
    // State
    this.state = {
      isLookingAtScreen: false,
      gazeStartTime: null,
      currentFace: null,
      faceHistory: [],
      historySize: 5,
      detectionSource: null,
      baselineFrame: null,
      lastFrameMetrics: null,
    };
    
    // Callbacks
    this.onGazeStart = null;
    this.onGazeEnd = null;
    this.onEngaged = null;
  }
  
  start() {
    this.isRunning = true;
    this.detect();
  }
  
  stop() {
    this.isRunning = false;
  }
  
  reset() {
    this.hasTriggeredWelcome = false;
    this.state.isLookingAtScreen = false;
    this.state.gazeStartTime = null;
    this.state.currentFace = null;
    this.state.faceHistory = [];
    this.state.detectionSource = null;
    this.state.baselineFrame = null;
    this.state.lastFrameMetrics = null;
  }
  
  async detect() {
    if (!this.isRunning) return;
    
    try {
      const frameMetrics = this.captureFrameMetrics();

      // Get face predictions
      const predictions = await this.model.estimateFaces(this.video, false);
      
      // Process predictions
      this.processPredictions(predictions, frameMetrics);
      
      // Visualize if debug mode is on
      if (debugMode) {
        this.visualizePredictions(predictions);
      }
      
    } catch (error) {
      console.error('Detection error:', error);
    }
    
    // Continue detection loop
    setTimeout(() => this.detect(), this.config.detectionInterval);
  }
  
  processPredictions(predictions, frameMetrics) {
    const now = Date.now();
    const obstructionPresent = this.isObstructionPresent(frameMetrics);
    
    if (predictions.length === 0 && !obstructionPresent) {
      this.updateBaselineFrame(frameMetrics);
      this.handleNoFace();
      return;
    }
    
    // Reset no-face timer since we detected a face or nearby obstruction
    if (typeof resetNoFaceTimer === 'function') {
      resetNoFaceTimer();
    }

    if (predictions.length === 0 && obstructionPresent) {
      this.state.currentFace = null;
      this.state.faceHistory = [];
      this.handleLooking(now, 'obstruction');
      return;
    }
    
    // Get primary face (largest)
    const primaryFace = this.getPrimaryFace(predictions);
    if (!primaryFace) {
      if (obstructionPresent) {
        this.handleLooking(now, 'obstruction');
      } else {
        this.updateBaselineFrame(frameMetrics);
        this.handleNoFace();
      }
      return;
    }
    
    // Calculate metrics
    const metrics = this.calculateMetrics(primaryFace);
    
    // Update history
    this.state.faceHistory.push(metrics);
    if (this.state.faceHistory.length > this.state.historySize) {
      this.state.faceHistory.shift();
    }
    
    this.state.currentFace = metrics;
    
    // Trigger on sustained face presence instead of gaze direction
    const isFacePresent = this.isFacePresent(metrics);
    
    if (isFacePresent) {
      this.handleLooking(now, 'face');
    } else if (obstructionPresent) {
      this.handleLooking(now, 'obstruction');
    } else {
      this.updateBaselineFrame(frameMetrics);
      this.handleNotLooking();
    }
  }

  captureFrameMetrics() {
    if (!this.analysisContext || !this.video.videoWidth || !this.video.videoHeight) {
      return null;
    }

    const width = this.config.analysisWidth;
    const height = this.config.analysisHeight;

    this.analysisContext.drawImage(this.video, 0, 0, width, height);
    const imageData = this.analysisContext.getImageData(0, 0, width, height).data;
    const grayscale = new Float32Array(width * height);

    let brightnessSum = 0;
    for (let i = 0, pixelIndex = 0; i < imageData.length; i += 4, pixelIndex += 1) {
      const brightness = (imageData[i] + imageData[i + 1] + imageData[i + 2]) / 3;
      grayscale[pixelIndex] = brightness;
      brightnessSum += brightness;
    }

    const averageBrightness = brightnessSum / grayscale.length;

    if (!this.state.baselineFrame) {
      this.state.baselineFrame = grayscale.slice();
      this.state.lastFrameMetrics = {
        averageBrightness,
        averageDiff: 0,
        changedCoverage: 0,
      };
      return this.state.lastFrameMetrics;
    }

    let diffSum = 0;
    let changedPixels = 0;
    for (let i = 0; i < grayscale.length; i += 1) {
      const diff = Math.abs(grayscale[i] - this.state.baselineFrame[i]);
      diffSum += diff;
      if (diff >= this.config.obstructionPixelThreshold) {
        changedPixels += 1;
      }
    }

    const metrics = {
      averageBrightness,
      averageDiff: diffSum / grayscale.length,
      changedCoverage: changedPixels / grayscale.length,
      grayscale,
    };

    this.state.lastFrameMetrics = metrics;
    return metrics;
  }

  isObstructionPresent(frameMetrics) {
    if (!frameMetrics) {
      return false;
    }

    const isVeryDark = frameMetrics.averageBrightness <= this.config.obstructionDarknessThreshold;
    const isLargeChange = (
      frameMetrics.averageDiff >= this.config.obstructionDiffThreshold &&
      frameMetrics.changedCoverage >= this.config.obstructionCoverageThreshold
    );

    return isVeryDark || isLargeChange;
  }

  updateBaselineFrame(frameMetrics) {
    if (!frameMetrics || !frameMetrics.grayscale) {
      return;
    }

    if (!this.state.baselineFrame) {
      this.state.baselineFrame = frameMetrics.grayscale.slice();
      return;
    }

    const nextBaseline = this.state.baselineFrame;
    const currentFrame = frameMetrics.grayscale;
    const rate = this.config.baselineLearningRate;

    for (let i = 0; i < currentFrame.length; i += 1) {
      nextBaseline[i] = (nextBaseline[i] * (1 - rate)) + (currentFrame[i] * rate);
    }
  }
  
  getPrimaryFace(predictions) {
    if (predictions.length === 0) return null;
    
    return predictions.reduce((largest, current) => {
      const largestArea = this.getFaceArea(largest);
      const currentArea = this.getFaceArea(current);
      return currentArea > largestArea ? current : largest;
    });
  }
  
  getFaceArea(face) {
    if (!face.topLeft || !face.bottomRight) return 0;
    const width = face.bottomRight[0] - face.topLeft[0];
    const height = face.bottomRight[1] - face.topLeft[1];
    return width * height;
  }
  
  calculateMetrics(face) {
    const videoWidth = this.video.videoWidth;
    const videoHeight = this.video.videoHeight;
    
    const topLeft = face.topLeft;
    const bottomRight = face.bottomRight;
    
    const faceWidth = bottomRight[0] - topLeft[0];
    const faceHeight = bottomRight[1] - topLeft[1];
    const faceCenterX = topLeft[0] + faceWidth / 2;
    const faceCenterY = topLeft[1] + faceHeight / 2;
    
    return {
      topLeft,
      bottomRight,
      width: faceWidth,
      height: faceHeight,
      centerX: faceCenterX / videoWidth,
      centerY: faceCenterY / videoHeight,
      size: (faceWidth * faceHeight) / (videoWidth * videoHeight),
      confidence: face.probability ? face.probability[0] : 1,
    };
  }
  
  isFacePresent(metrics) {
    // Check confidence
    if (metrics.confidence < this.config.minDetectionConfidence) {
      return false;
    }
    
    // Check size (person close enough)
    if (metrics.size < this.config.faceSizeThreshold) {
      return false;
    }

    return true;
  }
  
  handleLooking(now, source = 'face') {
    if (!this.state.isLookingAtScreen) {
      // Started looking
      this.state.isLookingAtScreen = true;
      this.state.gazeStartTime = now;
      this.state.detectionSource = source;
      
      if (this.onGazeStart) {
        this.onGazeStart();
      }
    } else {
      this.state.detectionSource = source;
      // Check duration
      const duration = now - this.state.gazeStartTime;
      
      if (duration >= this.config.requiredGazeDuration && !this.hasTriggeredWelcome) {
        this.hasTriggeredWelcome = true;
        
        if (this.onEngaged) {
          this.onEngaged({ duration, timestamp: now, source });
        }
      }
    }
  }
  
  handleNotLooking() {
    if (this.state.isLookingAtScreen) {
      this.state.isLookingAtScreen = false;
      this.state.gazeStartTime = null;
      this.state.detectionSource = null;
      
      if (this.onGazeEnd) {
        this.onGazeEnd();
      }
    }
  }
  
  handleNoFace() {
    this.handleNotLooking();
    this.state.currentFace = null;
    this.state.faceHistory = [];
    
    // Trigger no-face timer if not on default screen
    if (typeof startNoFaceTimer === 'function') {
      startNoFaceTimer();
    }
  }
  
  getGazeDuration() {
    if (!this.state.isLookingAtScreen || !this.state.gazeStartTime) {
      return 0;
    }
    return Date.now() - this.state.gazeStartTime;
  }
  
  visualizePredictions(predictions) {
    // Clear canvas
    overlayContext.clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);
    
    if (predictions.length === 0) return;
    
    predictions.forEach((prediction, i) => {
      const topLeft = prediction.topLeft;
      const bottomRight = prediction.bottomRight;
      const width = bottomRight[0] - topLeft[0];
      const height = bottomRight[1] - topLeft[1];
      
      // Draw bounding box
      overlayContext.strokeStyle = i === 0 ? '#F7A072' : '#2D5F7E';
      overlayContext.lineWidth = 3;
      overlayContext.strokeRect(topLeft[0], topLeft[1], width, height);
      
      // Draw landmarks if available
      if (prediction.landmarks) {
        overlayContext.fillStyle = '#E8C468';
        prediction.landmarks.forEach(landmark => {
          overlayContext.beginPath();
          overlayContext.arc(landmark[0], landmark[1], 3, 0, 2 * Math.PI);
          overlayContext.fill();
        });
      }
    });
  }
}
 
/**
 * Callback handlers
 */
function handleGazeStart() {
  console.log('User started looking at screen');
}
 
function handleGazeEnd() {
  console.log('User stopped looking at screen');
}
 
function handleUserEngaged(data) {
  console.log('User engaged!', data);
  
  // Only trigger welcome if on default screen
  if (currentScreen === 'defaultScreen' && !isVoiceAssistantActive()) {
    showWelcome();
    
    // Play welcome sound if available
    playWelcomeSound();
  }
}
 
/**
 * Initialize book returns system
 */
function initializeBookReturns() {
  // Inline BookReturnsManager class
  class BookReturnsManager {
    constructor(config = {}) {
      this.config = {
        maxBooksBeforeSort: config.maxBooksBeforeSort || 10,
        scanTimeout: config.scanTimeout || 30000,
      };
      
      this.state = {
        scannedBooks: [],
        totalBooksScanned: 0,
        totalBooksPlaced: 0,
        currentBook: null,
        isScanning: false,
      };
      
      this.onBookScanned = null;
      this.onBookPlaced = null;
      this.onThresholdReached = null;
    }
    
    startReturn() {
      this.state.isScanning = true;
      this.state.currentBook = null;
      console.log('Book return process started');
    }
    
    handleScan(barcode) {
      if (!this.state.isScanning) return null;
      
      const slot = this.assignSlot(barcode);
      const book = {
        barcode: barcode,
        slot: slot,
        scannedAt: new Date().toISOString(),
        placed: false,
      };
      
      this.state.currentBook = book;
      this.state.scannedBooks.push(book);
      this.state.totalBooksScanned++;
      
      console.log(`Book scanned: ${barcode} -> Slot ${slot}`);
      
      if (this.onBookScanned) {
        this.onBookScanned(book);
      }
      
      return book;
    }
    
    assignSlot(barcode) {
      const rows = ['A', 'B', 'C', 'D', 'E'];
      const columns = 10;
      const hash = barcode.split('').reduce((acc, char) => acc + char.charCodeAt(0), 0);
      const row = rows[hash % rows.length];
      const col = (hash % columns) + 1;
      return `${row}${col}`;
    }
    
    markBookPlaced() {
      if (!this.state.currentBook) return;
      
      this.state.currentBook.placed = true;
      this.state.currentBook.placedAt = new Date().toISOString();
      this.state.totalBooksPlaced++;
      
      console.log(`Book placed in slot ${this.state.currentBook.slot}`);
      
      if (this.onBookPlaced) {
        this.onBookPlaced(this.state.currentBook);
      }
      
      if (this.state.totalBooksPlaced >= this.config.maxBooksBeforeSort) {
        this.handleThresholdReached();
      }
      
      this.state.currentBook = null;
    }
    
    handleThresholdReached() {
      console.log(`Threshold reached: ${this.state.totalBooksPlaced} books placed`);
      
      if (this.onThresholdReached) {
        this.onThresholdReached({
          totalBooks: this.state.totalBooksPlaced,
          books: this.state.scannedBooks.filter(b => b.placed)
        });
      }
      
      this.triggerSortingProcess();
    }
    
    triggerSortingProcess() {
      console.log('═══════════════════════════════════════');
      console.log('🤖 SORTING PROCESS TRIGGERED');
      console.log(`📚 Processing ${this.state.totalBooksPlaced} books...`);
      console.log('═══════════════════════════════════════');
      
      // Show alert to user
      alert(`Sorting process triggered!\n\n${this.state.totalBooksPlaced} books will now be processed and sorted.\n\nCounter will reset after sorting completes.`);
      
      // TODO: Add your sorting mechanism here
      // - Send data to backend API
      // - Trigger robot/conveyor system
      // - Update database
      // - etc.
      
      // Reset counter ONLY after sorting is triggered
      this.resetAfterSort();
      
      console.log('✅ Counter reset - ready for next batch of books');
    }
    
    resetAfterSort() {
      // Only reset the books that have been placed
      // Keep the scanned books list for ongoing session tracking if needed
      this.state.scannedBooks = this.state.scannedBooks.filter(b => !b.placed);
      this.state.totalBooksPlaced = 0;
      console.log('Placed books counter reset after sorting - ready for next batch');
    }
    
    getStats() {
      return {
        totalScanned: this.state.totalBooksScanned,
        totalPlaced: this.state.totalBooksPlaced,
        currentBook: this.state.currentBook,
        isScanning: this.state.isScanning,
      };
    }
    
    reset() {
      this.state = {
        scannedBooks: [],
        totalBooksScanned: 0,
        totalBooksPlaced: 0,
        currentBook: null,
        isScanning: false,
      };
    }
  }
  
  // Create instance
  bookReturnsManager = new BookReturnsManager({
    maxBooksBeforeSort: 10, // Trigger sorting after 10 books
  });
  
  // IMPORTANT: Counter behavior
  // - totalBooksScanned: Resets for each new user (shows current user's scans)
  // - totalBooksPlaced: Persists across users until threshold reached
  //
  // Example:
  // - User A scans 3 books, places 3 → Scanned: 3, Placed: 3/10
  // - User A leaves → Scanned resets to 0, Placed stays at 3
  // - User B scans 5 books, places 4 → Scanned: 5, Placed: 7/10
  // - User B leaves → Scanned resets to 0, Placed stays at 7
  // - User C scans 4 books, places 3 → Scanned: 4, Placed: 10/10
  // - SORTING TRIGGERED → Both counters reset to 0
  
  // Set up callbacks
  bookReturnsManager.onBookScanned = (book) => {
    updateScanStats();
    showPlacementScreen(book.slot);
  };
  
  bookReturnsManager.onBookPlaced = (book) => {
    updateScanStats();
    // Return to scan screen for next book
    setTimeout(() => {
      switchScreen('bookReturnsScanScreen');
    }, 1500);
  };
  
  bookReturnsManager.onThresholdReached = (data) => {
    console.log('Threshold reached callback:', data);
  };
  
  console.log('Book returns system initialized');
}

/**
 * Initialize self checkout system
 */
function initializeSelfCheckout() {
  class SelfCheckoutManager {
    constructor(config = {}) {
      this.config = {
        loanPeriodDays: config.loanPeriodDays || 14,
      };

      this.reset();
    }

    startSession(patron) {
      this.state.patron = patron;
      this.state.scannedBooks = [];
      this.state.isScanning = true;
    }

    scanBook(scanCode) {
      if (!this.state.isScanning) {
        return { error: 'Start a checkout session before scanning.' };
      }

      const normalizedCode = String(scanCode || '').trim();
      if (!normalizedCode) {
        return { error: 'Please scan a barcode or enter an ISBN.' };
      }

      const duplicate = this.state.scannedBooks.some((book) => (
        book.scanCode === normalizedCode || book.isbn === normalizedCode
      ));
      if (duplicate) {
        return { error: 'That book has already been scanned for this checkout.' };
      }

      const book = this.findBookByCode(normalizedCode);
      if (!book) {
        return { error: `No catalog match was found for code ${normalizedCode}.` };
      }

      if (book.status !== 'available') {
        return { error: `"${book.title}" is currently ${formatBookStatus(book.status)}.` };
      }

      const dueDate = this.buildDueDate();
      const scannedBook = {
        id: book.id,
        scanCode: normalizedCode,
        isbn: book.isbn,
        title: book.title,
        author: book.author,
        dueDate,
      };

      this.state.scannedBooks.push(scannedBook);
      return { book: scannedBook };
    }

    completeSession() {
      if (!this.state.isScanning) {
        return { error: 'No active checkout session was found.' };
      }

      if (this.state.scannedBooks.length === 0) {
        return { error: 'Scan at least one book before completing checkout.' };
      }

      const summary = {
        patron: { ...this.state.patron },
        books: this.state.scannedBooks.map((book) => ({ ...book })),
        dueDate: this.state.scannedBooks[0].dueDate,
      };

      summary.books.forEach((scannedBook) => {
        const catalogBook = booksDatabase.find((book) => book.id === scannedBook.id);
        if (catalogBook) {
          catalogBook.status = 'checked-out';
          catalogBook.dueDate = scannedBook.dueDate;
        }
      });

      this.reset();
      return { summary };
    }

    findBookByCode(scanCode) {
      const normalizedCode = scanCode.toLowerCase();

      return booksDatabase.find((book) => {
        const possibleCodes = [book.isbn, book.id, book.barcode]
          .filter(Boolean)
          .map((value) => String(value).toLowerCase());

        return possibleCodes.includes(normalizedCode);
      });
    }

    buildDueDate() {
      const dueDate = new Date();
      dueDate.setDate(dueDate.getDate() + this.config.loanPeriodDays);
      return dueDate.toISOString().split('T')[0];
    }

    getStats() {
      return {
        patron: this.state.patron,
        totalScanned: this.state.scannedBooks.length,
        scannedBooks: this.state.scannedBooks.map((book) => ({ ...book })),
        dueDate: (this.state.scannedBooks[0] && this.state.scannedBooks[0].dueDate) || null,
        isScanning: this.state.isScanning,
      };
    }

    reset() {
      this.state = {
        patron: null,
        scannedBooks: [],
        isScanning: false,
      };
    }
  }

  selfCheckoutManager = new SelfCheckoutManager({
    loanPeriodDays: 14,
  });

  updateSelfCheckoutUI();
  console.log('Self checkout system initialized');
}
 
/**
 * Book Returns Functions
 */
function startBookReturns() {
  // Reset scanned count for new user session
  // But keep totalBooksPlaced - it persists until threshold
  bookReturnsManager.state.totalBooksScanned = 0;
  bookReturnsManager.state.scannedBooks = bookReturnsManager.state.scannedBooks.filter(b => b.placed);
  
  // Start new scanning session
  bookReturnsManager.startReturn();
  updateScanStats();
  switchScreen('bookReturnsScanScreen');
  resetInactivityTimer();
}
 
function simulateScan() {
  // Generate random barcode
  const barcode = 'BOOK' + Math.floor(Math.random() * 1000000).toString().padStart(6, '0');
  bookReturnsManager.handleScan(barcode);
  resetInactivityTimer();
}
 
function markBookPlaced() {
  bookReturnsManager.markBookPlaced();
  resetInactivityTimer();
}
 
function showPlacementScreen(slot) {
  document.getElementById('assignedSlot').textContent = slot;
  updateScanStats();
  switchScreen('bookReturnsPlaceScreen');
}
 
function updateScanStats() {
  const stats = bookReturnsManager.getStats();
  const maxBooks = bookReturnsManager.config.maxBooksBeforeSort;
  const remaining = maxBooks - stats.totalPlaced;
  const progress = (stats.totalPlaced / maxBooks) * 100;
  
  // Update scan screen stats
  const scannedCount = document.getElementById('scannedCount');
  const placedCount = document.getElementById('placedCount');
  const remainingCount = document.getElementById('remainingCount');
  const thresholdFill = document.getElementById('thresholdFill');
  
  if (scannedCount) scannedCount.textContent = stats.totalScanned;
  if (placedCount) placedCount.textContent = stats.totalPlaced;
  if (remainingCount) remainingCount.textContent = remaining;
  if (thresholdFill) thresholdFill.style.width = progress + '%';
  
  // Update placement screen stats
  const scannedCountPlace = document.getElementById('scannedCountPlace');
  const placedCountPlace = document.getElementById('placedCountPlace');
  const remainingCountPlace = document.getElementById('remainingCountPlace');
  const thresholdFillPlace = document.getElementById('thresholdFillPlace');
  
  if (scannedCountPlace) scannedCountPlace.textContent = stats.totalScanned;
  if (placedCountPlace) placedCountPlace.textContent = stats.totalPlaced;
  if (remainingCountPlace) remainingCountPlace.textContent = remaining;
  if (thresholdFillPlace) thresholdFillPlace.style.width = progress + '%';
}
 
function exitBookReturns() {
  // Reset scanned count for this user's session
  // But keep totalBooksPlaced - it persists until threshold
  bookReturnsManager.state.totalBooksScanned = 0;
  bookReturnsManager.state.scannedBooks = bookReturnsManager.state.scannedBooks.filter(b => b.placed);
  bookReturnsManager.state.isScanning = false;
  bookReturnsManager.state.currentBook = null;
  returnToDefault();
}

/**
 * Self Checkout Functions
 */
function startSelfCheckout() {
  if (selfCheckoutManager) {
    selfCheckoutManager.reset();
  }

  const nameInput = document.getElementById('checkoutNameInput');
  const idInput = document.getElementById('checkoutIdInput');
  const scanInput = document.getElementById('checkoutScanInput');

  if (nameInput) nameInput.value = '';
  if (idInput) idInput.value = '';
  if (scanInput) scanInput.value = '';

  updateSelfCheckoutUI();
  setSelfCheckoutStatus('Enter your name and ID number to begin.', 'info');
  switchScreen('selfCheckoutPatronScreen');
  resetInactivityTimer();

  setTimeout(() => {
    if (nameInput) {
      nameInput.focus();
    }
  }, 300);
}

function beginSelfCheckout() {
  const nameInput = document.getElementById('checkoutNameInput');
  const idInput = document.getElementById('checkoutIdInput');
  const patronName = nameInput ? nameInput.value.trim() : '';
  const patronId = idInput ? idInput.value.trim() : '';

  if (!patronName || !patronId) {
    alert('Please enter both your name and ID number before continuing.');
    resetInactivityTimer();
    return;
  }

  selfCheckoutManager.startSession({
    name: patronName,
    idNumber: patronId,
  });

  updateSelfCheckoutUI();
  setSelfCheckoutStatus('Waiting for the first scan...', 'info');
  switchScreen('selfCheckoutScanScreen');
  resetInactivityTimer();

  setTimeout(() => {
    const scanInput = document.getElementById('checkoutScanInput');
    if (scanInput) {
      scanInput.focus();
    }
  }, 300);
}

function handleSelfCheckoutScan(scanCode = null) {
  if (!selfCheckoutManager) {
    return;
  }

  const scanInput = document.getElementById('checkoutScanInput');
  const valueToScan = scanCode || (scanInput ? scanInput.value : '');
  const result = selfCheckoutManager.scanBook(valueToScan);

  if (result.error) {
    setSelfCheckoutStatus(result.error, 'error');
    resetInactivityTimer();
    return;
  }

  if (scanInput) {
    scanInput.value = '';
    scanInput.focus();
  }

  updateSelfCheckoutUI();
  setSelfCheckoutStatus(`Added "${result.book.title}" to this checkout.`, 'success');
  resetInactivityTimer();
}

function simulateCheckoutScan() {
  if (!selfCheckoutManager) {
    return;
  }

  const scannedIsbns = new Set(
    selfCheckoutManager.getStats().scannedBooks.map((book) => book.isbn)
  );
  const availableBook = booksDatabase.find((book) => (
    book.status === 'available' && !scannedIsbns.has(book.isbn)
  ));

  if (!availableBook) {
    setSelfCheckoutStatus('No available books are left to simulate.', 'error');
    resetInactivityTimer();
    return;
  }

  handleSelfCheckoutScan(availableBook.isbn);
}

function completeSelfCheckout() {
  if (!selfCheckoutManager) {
    return;
  }

  const result = selfCheckoutManager.completeSession();
  if (result.error) {
    setSelfCheckoutStatus(result.error, 'error');
    resetInactivityTimer();
    return;
  }

  updateSelfCheckoutUI();
  alert(
    `${result.summary.books.length} book(s) checked out to ${result.summary.patron.name}.\n\nDue date: ${formatDate(result.summary.dueDate)}`
  );
  showThankYou();
}

function exitSelfCheckout() {
  if (selfCheckoutManager) {
    selfCheckoutManager.reset();
  }

  const nameInput = document.getElementById('checkoutNameInput');
  const idInput = document.getElementById('checkoutIdInput');
  const scanInput = document.getElementById('checkoutScanInput');

  if (nameInput) nameInput.value = '';
  if (idInput) idInput.value = '';
  if (scanInput) scanInput.value = '';

  updateSelfCheckoutUI();
  returnToDefault();
}

function updateSelfCheckoutUI() {
  const stats = selfCheckoutManager ? selfCheckoutManager.getStats() : {
    patron: null,
    totalScanned: 0,
    scannedBooks: [],
    dueDate: null,
    isScanning: false,
  };

  const patronSummary = document.getElementById('checkoutPatronSummary');
  const scannedCount = document.getElementById('checkoutScannedCount');
  const dueDate = document.getElementById('checkoutDueDate');
  const scannedBooks = document.getElementById('checkoutScannedBooks');

  if (patronSummary) {
    patronSummary.textContent = stats.patron
      ? `Patron: ${stats.patron.name} • ID: ${stats.patron.idNumber}`
      : 'Patron: Not started';
  }

  if (scannedCount) {
    scannedCount.textContent = String(stats.totalScanned);
  }

  if (dueDate) {
    dueDate.textContent = stats.dueDate ? formatDate(stats.dueDate) : '--';
  }

  if (scannedBooks) {
    if (stats.scannedBooks.length === 0) {
      scannedBooks.innerHTML = '<div class="checkout-empty">No books scanned yet.</div>';
    } else {
      scannedBooks.innerHTML = stats.scannedBooks.map(createCheckoutBookCard).join('');
    }
  }
}

function createCheckoutBookCard(book) {
  return `
    <div class="checkout-book-item">
      <div class="checkout-book-title">${escapeHtml(book.title)}</div>
      <div class="checkout-book-meta">by ${escapeHtml(book.author)}</div>
      <div class="checkout-book-meta">ISBN: ${escapeHtml(book.isbn)}</div>
      <div class="checkout-book-due">Due ${escapeHtml(formatDate(book.dueDate))}</div>
    </div>
  `;
}

function setSelfCheckoutStatus(message, type = 'info') {
  const status = document.getElementById('checkoutScanStatus');
  if (!status) {
    return;
  }

  status.textContent = message;
  status.className = 'checkout-status-message';

  if (type === 'error' || type === 'success') {
    status.classList.add(type);
  }
}

function formatBookStatus(status) {
  if (status === 'checked-out') return 'checked out and unavailable';
  if (status === 'on-hold') return 'on hold and unavailable';
  return status.replace(/-/g, ' ');
}

/**
 * Voice Assistant Functions
 */
/**
 * Whisper-based fallback for browsers without Web Speech API (Firefox).
 * Records audio via MediaRecorder, sends to /api/transcribe (OpenAI Whisper).
 * Mimics the SpeechRecognition interface so the rest of the code works unchanged.
 */
class WhisperRecognition {
  constructor() {
    this.onstart = null;
    this.onresult = null;
    this.onerror = null;
    this.onend = null;
    this._recording = false;
    this._stream = null;
    this._recorder = null;
    this._silenceTimer = null;
    this._analyser = null;
    this._silenceStart = 0;
    this._speechDetected = false;
    // How long silence after speech before we auto-stop and transcribe (ms)
    this._silenceThreshold = 1400;
    // RMS below this = silence (raise if picking up background noise)
    this._noiseGate = 0.06;
  }

  async start() {
    // Already recording — don't double-start
    if (this._recording) return;
    try {
      this._stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      const actx = new AudioContext();
      const src = actx.createMediaStreamSource(this._stream);
      this._analyser = actx.createAnalyser();
      this._analyser.fftSize = 256;
      src.connect(this._analyser);

      this._recorder = new MediaRecorder(this._stream, { mimeType: 'audio/webm' });
      const chunks = [];
      this._recorder.ondataavailable = (e) => { if (e.data.size > 0) chunks.push(e.data); };
      this._recorder.onstop = async () => {
        if (chunks.length === 0 || !this._speechDetected) {
          chunks.length = 0;
          // If paused (TTS playing), just wait — resumeListening will restart
          if (this._paused) return;
          if (this._recording) { this._recorder.start(); this._watchSilence(); }
          else { this._finish(); }
          return;
        }
        const blob = new Blob(chunks, { type: 'audio/webm' });
        chunks.length = 0;
        await this._transcribe(blob);
        // Fire onend so the app processes the transcript (handleVoiceAssistantPrompt)
        if (this.onend) this.onend();
        // If still in continuous mode and not paused (TTS playing), restart
        if (this._recording && !this._paused) {
          this._recorder.start();
          this._watchSilence();
          if (this.onstart) this.onstart();
        }
      };

      this._recording = true;
      this._recorder.start();
      this._watchSilence();
      if (this.onstart) this.onstart();
    } catch (err) {
      if (this.onerror) this.onerror({ error: err.name === 'NotAllowedError' ? 'not-allowed' : 'audio-capture' });
    }
  }

  stop() {
    this._recording = false;
    clearInterval(this._silenceTimer);
    if (this._recorder && this._recorder.state === 'recording') {
      this._recorder.stop();
    } else {
      this._finish();
    }
  }

  // Pause mic (while TTS is playing) — stops recorder to flush audio buffer
  pauseListening() {
    this._paused = true;
    this._speechDetected = false;
    clearInterval(this._silenceTimer);
    // Stop recorder so TTS audio doesn't accumulate in chunks
    if (this._recorder && this._recorder.state === 'recording') {
      this._recorder.stop(); // onstop will see _speechDetected=false, _paused=true → discard & wait
    }
  }

  resumeListening() {
    if (!this._recording) return;
    var self = this;
    // Keep _paused = true so any pending onstop from the old recorder discards & returns.
    // Only clear _paused and restart once the echo window has passed.
    setTimeout(function() {
      self._paused = false;
      if (!self._recording || !self._stream) return;
      self._speechDetected = false;
      if (self._recorder && self._recorder.state !== 'recording') {
        self._recorder.start();
        self._watchSilence();
        if (self.onstart) self.onstart();
      }
    }, 500);
  }

  _finish() {
    if (this._stream) { this._stream.getTracks().forEach(t => t.stop()); this._stream = null; }
    if (this.onend) this.onend();
  }

  _watchSilence() {
    clearInterval(this._silenceTimer);
    this._silenceStart = Date.now();
    this._speechDetected = false;
    const buf = new Uint8Array(this._analyser.frequencyBinCount);
    this._silenceTimer = setInterval(() => {
      this._analyser.getByteTimeDomainData(buf);
      let sum = 0;
      for (let i = 0; i < buf.length; i++) { const v = (buf[i] - 128) / 128; sum += v * v; }
      const rms = Math.sqrt(sum / buf.length);
      if (rms > this._noiseGate) {
        // Actual speech detected
        this._speechDetected = true;
        this._silenceStart = Date.now();
      }
      // Only trigger transcription after speech was detected then went silent
      if (this._speechDetected && Date.now() - this._silenceStart > this._silenceThreshold) {
        clearInterval(this._silenceTimer);
        if (this._recorder && this._recorder.state === 'recording') {
          this._recorder.stop(); // triggers onstop → transcribe
        }
      }
    }, 100);
  }

  async _transcribe(blob) {
    try {
      const resp = await fetch('/api/transcribe', { method: 'POST', body: blob });
      const data = await resp.json();
      if (data.text && data.text.trim()) {
        // Normalize common Whisper mishearings of "Libot"
        var text = data.text.trim().replace(/\b(lie[\s-]?bot|ly[\s-]?bot|li[\s-]?bot|lye[\s-]?bot|live[\s-]?bot|libott?|lye[\s-]?bo|lie[\s-]?bo|live[\s-]?bo|ly[\s-]?bo|light[\s-]?bulb|light[\s-]?bot|light[\s-]?bott?|lai[\s-]?bot|la[iy][\s-]?bo[t]?|live[\s-]?up|libo|lybo|labou?t|leib[ao]|leyb[ao]|l[ae]i[\s-]?b[aou]t?)\b/gi, 'Libot');
        // Simulate a SpeechRecognition result event
        if (this.onresult) {
          this.onresult({
            results: [{ 0: { transcript: text }, isFinal: true, length: 1 }],
          });
        }
      }
    } catch (err) {
      console.warn('Whisper transcription failed:', err);
      if (this.onerror) this.onerror({ error: 'network' });
    }
  }
}

function initializeVoiceAssistant() {
  const NativeSpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
  // Use native Web Speech API if available, otherwise fall back to Whisper
  const useWhisper = !NativeSpeechRecognition;
  speechRecognitionSupported = Boolean(NativeSpeechRecognition || typeof MediaRecorder !== 'undefined');

  if (!speechRecognitionSupported) {
    updateVoiceAssistantUI({
      status: 'Voice chat is not available in this browser.',
      message: 'Neither Web Speech API nor MediaRecorder is available.',
      transcript: 'Voice recognition unavailable.',
      buttonLabel: 'Voice Unavailable',
      listening: false,
      speaking: false,
    });
    return;
  }

  if (useWhisper) {
    console.log('Web Speech API not available — using Whisper fallback via /api/transcribe');
    speechRecognition = new WhisperRecognition();
  } else {
    speechRecognition = new NativeSpeechRecognition();
    speechRecognition.lang = 'en-US';
    speechRecognition.interimResults = true;
    speechRecognition.continuous = true;
    speechRecognition.maxAlternatives = 1;
  }

  speechRecognition.onstart = () => {
    manualVoiceStopRequested = false;
    isVoiceListening = true;
    lastVoiceRecognitionError = '';
    updateVoiceAssistantUI({
      status: 'Listening...',
      message: 'I am listening. Ask me a library question.',
      transcript: 'Go ahead, I am ready.',
      buttonLabel: 'Stop',
      listening: true,
      speaking: false,
    });
  };

  speechRecognition.onresult = (event) => {
    const transcript = Array.from(event.results)
      .map((result) => (result[0] && result[0].transcript) || '')
      .join(' ')
      .trim();

    if (!transcript) {
      return;
    }

    updateVoiceAssistantUI({
      status: event.results[event.results.length - 1].isFinal ? 'Thinking...' : 'Listening...',
      message: event.results[event.results.length - 1].isFinal
        ? 'Let me think about that.'
        : 'I am still listening.',
      transcript: `You said: "${transcript}"`,
      buttonLabel: 'Stop',
      listening: true,
      speaking: false,
    });

    if (event.results[event.results.length - 1].isFinal) {
      pendingVoiceResponse = transcript;
    }
  };

  speechRecognition.onerror = (event) => {
    isVoiceListening = false;
    lastVoiceRecognitionError = event.error;

    if (manualVoiceStopRequested) {
      return;
    }

    const recoverableErrors = ['no-speech', 'aborted'];
    if (voiceSessionActive && recoverableErrors.includes(event.error)) {
      updateVoiceAssistantUI({
        status: 'Listening...',
        message: event.error === 'no-speech'
          ? 'I did not hear anything yet, but I am still listening.'
          : 'The microphone session was interrupted. I am reconnecting now.',
        transcript: `Debug: recognition error "${event.error}" - auto retrying.`,
        buttonLabel: 'Stop',
        listening: false,
        speaking: false,
      });
      scheduleVoiceRecognitionRestart();
      return;
    }

    manualVoiceStopRequested = false;
    voiceSessionActive = false;
    clearVoiceRestartTimeout();
    const message = event.error === 'not-allowed'
      ? 'Microphone access was blocked. Please allow microphone access to use voice chat.'
      : event.error === 'audio-capture'
        ? 'No microphone input was found. Please check that a microphone is connected and allowed.'
        : 'I ran into a microphone issue. Please try again.';

    updateVoiceAssistantUI({
      status: 'Voice Error',
      message,
      transcript: `Debug: recognition error "${event.error}"`,
      buttonLabel: 'Speak',
      listening: false,
      speaking: false,
    });
  };

  speechRecognition.onend = () => {
    const latestTranscript = pendingVoiceResponse;
    isVoiceListening = false;
    console.log('onend fired — transcript:', latestTranscript ? latestTranscript.substring(0, 30) : '(none)',
      'manual:', manualVoiceStopRequested, 'session:', voiceSessionActive,
      'speaking:', isVoiceSpeaking, 'history:', voiceConversationHistory.length);

    if (manualVoiceStopRequested) {
      manualVoiceStopRequested = false;
      voiceSessionActive = false;
      clearVoiceRestartTimeout();
      updateVoiceAssistantUI({
        status: 'Ready to talk',
        message: 'Tap Speak whenever you would like to talk.',
        transcript: 'Listening stopped.',
        buttonLabel: 'Speak',
        listening: false,
        speaking: false,
      });
      return;
    }

    if (latestTranscript) {
      pendingVoiceResponse = '';
      // Don't launch a second response while one is in flight
      if (isVoiceProcessing) {
        console.log('onend: skipping — already processing a response');
        return;
      }
      // Reject echo: ignore transcripts arriving shortly after TTS finished
      if (_lastTtsEndTime && (Date.now() - _lastTtsEndTime) < 2000) {
        console.log('onend: skipping — likely TTS echo (within 2s of TTS end)');
        return;
      }
      handleVoiceAssistantPrompt(latestTranscript);
      scheduleVoiceRecognitionRestart();
      return;
    }

    if (voiceSessionActive && !isVoiceSpeaking) {
      updateVoiceAssistantUI({
        status: 'Listening...',
        message: 'I am still listening. Click the button whenever you want me to stop.',
        transcript: lastVoiceRecognitionError
          ? `Debug: recognition ended after "${lastVoiceRecognitionError}". Waiting for your next question...`
          : 'Waiting for your next question...',
        buttonLabel: 'Stop',
        listening: false,
        speaking: false,
      });
      scheduleVoiceRecognitionRestart();
      return;
    }

    if (!isVoiceSpeaking) {
      updateVoiceAssistantUI({
        status: 'Ready to talk',
        message: 'Tap Speak whenever you would like to talk.',
        transcript: 'Listening stopped.',
        buttonLabel: 'Speak',
        listening: false,
        speaking: false,
      });
    }
  };

  updateVoiceAssistantUI({
    status: 'Ready to talk',
    message: 'Tap Speak and I will listen like a friendly library robot.',
    transcript: 'Listening transcript will appear here.',
    buttonLabel: 'Speak',
    listening: false,
    speaking: false,
  });
}

function toggleVoiceAssistant() {
  if (!speechRecognitionSupported || !speechRecognition) {
    updateVoiceAssistantUI({
      status: 'Voice Unavailable',
      message: 'Voice chat needs browser speech recognition and speech synthesis support.',
      transcript: 'Try Chrome or Edge for this proof of concept.',
      buttonLabel: 'Voice Unavailable',
      listening: false,
      speaking: false,
    });
    return;
  }

  resetInactivityTimer();
  console.log('toggleVoiceAssistant — listening:', isVoiceListening, 'speaking:', isVoiceSpeaking,
    'session:', voiceSessionActive, 'recording:', speechRecognition._recording, 'history:', voiceConversationHistory.length);

  if (isVoiceListening) {
    console.log('  → STOP branch');
    manualVoiceStopRequested = true;
    voiceSessionActive = false;
    clearVoiceRestartTimeout();
    speechRecognition.stop();
    return;
  }

  if (isVoiceSpeaking) {
    console.log('  → cancel speaking');
    if (_ttsAudio) { _ttsAudio.pause(); _ttsAudio = null; }
    if (window.speechSynthesis) window.speechSynthesis.cancel();
    isVoiceSpeaking = false;
    isVoiceProcessing = false;
  }

  pendingVoiceResponse = '';
  manualVoiceStopRequested = false;
  voiceSessionActive = true;
  // Don't clear history — preserve context across Speak presses
  resetVoiceHistoryTimeout();
  clearVoiceRestartTimeout();
  speechRecognition.start();
}

// Active TTS audio so we can cancel it
let _ttsAudio = null;

function speakRobotResponse(response, transcript) {
  // Cancel any in-progress TTS
  if (_ttsAudio) { _ttsAudio.pause(); _ttsAudio = null; }
  if (window.speechSynthesis) window.speechSynthesis.cancel();

  // Pause mic so TTS output doesn't trigger self-interruption
  if (speechRecognition && speechRecognition.pauseListening) {
    speechRecognition.pauseListening();
  }

  isVoiceSpeaking = true;
  updateVoiceAssistantUI({
    status: 'Speaking...',
    message: response,
    transcript: 'You said: "' + transcript + '"',
    buttonLabel: voiceSessionActive ? 'Stop' : 'Speak Again',
    listening: false,
    speaking: true,
  });

  function onDone() {
    _ttsAudio = null;
    isVoiceSpeaking = false;
    isVoiceProcessing = false;
    // Resume mic after delay so speaker echo fades
    setTimeout(function() {
      if (speechRecognition && speechRecognition.resumeListening) {
        speechRecognition.resumeListening();
      }
    }, 600);
    updateVoiceAssistantUI({
      status: voiceSessionActive ? 'Listening...' : 'Ready to talk',
      message: response,
      transcript: 'You said: "' + transcript + '"',
      buttonLabel: voiceSessionActive ? 'Stop' : 'Speak Again',
      listening: false,
      speaking: false,
    });
    if (voiceSessionActive) scheduleVoiceRecognitionRestart();
  }

  function onError() {
    _ttsAudio = null;
    isVoiceSpeaking = false;
    isVoiceProcessing = false;
    if (speechRecognition && speechRecognition.resumeListening) {
      speechRecognition.resumeListening();
    }
    updateVoiceAssistantUI({
      status: 'Voice Error',
      message: response,
      transcript: 'You said: "' + transcript + '"',
      buttonLabel: 'Speak Again',
      listening: false,
      speaking: false,
    });
  }

  // OpenAI TTS via server — no browser fallback
  fetch('/api/tts', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ text: response }),
  })
    .then(function(res) {
      if (!res.ok) throw new Error('TTS request failed');
      return res.blob();
    })
    .then(function(blob) {
      var url = URL.createObjectURL(blob);
      var audio = new Audio(url);
      _ttsAudio = audio;
      audio.onended = function() { URL.revokeObjectURL(url); onDone(); };
      audio.onerror = function() { URL.revokeObjectURL(url); onError(); };
      audio.play();
    })
    .catch(function(err) {
      console.warn('OpenAI TTS failed:', err);
      onError();
    });
}

function updateVoiceAssistantUI({
  status,
  message,
  transcript,
  buttonLabel,
  listening = false,
  speaking = false,
}) {
  if (elements.robotStatusText && status) {
    elements.robotStatusText.textContent = status;
  }

  if (elements.voiceAssistantMessage && message) {
    elements.voiceAssistantMessage.textContent = message;
  }

  if (elements.voiceTranscriptText && transcript) {
    elements.voiceTranscriptText.textContent = transcript;
  }

  if (elements.speakButton && buttonLabel) {
    elements.speakButton.textContent = buttonLabel;
  }

  if (elements.robotFace) {
    elements.robotFace.classList.toggle('listening', listening);
    elements.robotFace.classList.toggle('speaking', speaking);
  }

  // Drive voice bars from mic/sine waves instead of uniform CSS
  if (listening) {
    VoiceVisualizer.startListening();
  } else if (speaking) {
    VoiceVisualizer.startSpeaking();
  } else {
    VoiceVisualizer.stop();
  }
}

function generateRobotVoiceReply(transcript) {
  const lower = transcript.toLowerCase();

  if (matchesAny(lower, ['hello', 'hi', 'hey', 'good morning', 'good afternoon'])) {
    return 'Hello there. I am your friendly library robot. I can help with returns, self checkout, book status, directions, printing, and general library questions.';
  }

  if (matchesAny(lower, ['hours', 'open', 'close', 'closing time'])) {
    return 'I am only a proof of concept robot, so I do not have live hours yet. Please check the front desk or the library website for today\'s schedule.';
  }

  if (matchesAny(lower, ['return', 'book return', 'drop off'])) {
    return 'You can use the Book Returns option on the menu. Scan each item and I will guide you through the return flow.';
  }

  if (matchesAny(lower, ['checkout', 'borrow', 'check out'])) {
    return 'You can choose Self Checkout from the menu. I will ask for your name and ID number, then help you scan your books.';
  }

  if (matchesAny(lower, ['status', 'find a book', 'search', 'where is my book', 'book status'])) {
    return 'Try the Check Book Status option from the menu. You can search by title, author, or ISBN.';
  }

  if (matchesAny(lower, ['print', 'printer', 'printing'])) {
    return 'For printing help, I can point you toward the print room and the front desk can help with account or payment issues.';
  }

  if (matchesAny(lower, ['bathroom', 'restroom', 'where is'])) {
    return 'I can help with directions in a future version. For now, please ask the front desk for the fastest route.';
  }

  if (matchesAny(lower, ['thank you', 'thanks'])) {
    return 'You are very welcome. I am happy to help.';
  }

  if (matchesAny(lower, ['who are you', 'your name', 'what are you'])) {
    return 'I am a friendly library robot prototype. I listen, respond, and help visitors get to the right library service.';
  }

  return 'I heard you, and I am still learning. I can best help with book returns, self checkout, book status, printing, and basic library guidance.';
}

function matchesAny(text, phrases) {
  return phrases.some((phrase) => text.includes(phrase));
}

async function handleVoiceAssistantPrompt(transcript) {
  isVoiceProcessing = true;
  updateVoiceAssistantUI({
    status: 'Thinking...',
    message: 'Let me think about that.',
    transcript: 'You said: "' + transcript + '"',
    buttonLabel: voiceSessionActive ? 'Stop' : 'Speak Again',
    listening: false,
    speaking: false,
  });

  // Pause mic so TTS doesn't trigger self-interruption
  if (speechRecognition && speechRecognition.pauseListening) {
    speechRecognition.pauseListening();
  }

  // Stream sentences from chatbot, prefetch TTS audio, play sequentially
  var fullReply = '';
  var audioQueue = []; // queue of promises that resolve to audio blob URLs
  var isPlaying = false;
  var streamDone = false;
  var firstSentence = true;

  function prefetchTTS(sentence) {
    // Start fetching TTS immediately, return a promise for the blob URL
    var p = fetch('/api/tts', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ text: sentence }),
    })
      .then(function(r) {
        if (!r.ok) throw new Error('TTS failed');
        return r.blob();
      })
      .then(function(blob) {
        return URL.createObjectURL(blob);
      });
    audioQueue.push(p);
  }

  function playNext() {
    if (isPlaying || audioQueue.length === 0) {
      if (streamDone && audioQueue.length === 0 && !isPlaying) {
        onAllDone();
      }
      return;
    }
    isPlaying = true;
    var nextPromise = audioQueue.shift();
    nextPromise
      .then(function(url) {
        var audio = new Audio(url);
        _ttsAudio = audio;
        audio.onended = function() {
          URL.revokeObjectURL(url);
          isPlaying = false;
          _ttsAudio = null;
          playNext();
        };
        audio.onerror = function() {
          URL.revokeObjectURL(url);
          isPlaying = false;
          _ttsAudio = null;
          playNext();
        };
        audio.play();
      })
      .catch(function() {
        isPlaying = false;
        playNext();
      });
  }

  function onAllDone() {
    isVoiceSpeaking = false;
    isVoiceProcessing = false;
    _ttsAudio = null;
    _lastTtsEndTime = Date.now();
    setTimeout(function() {
      if (speechRecognition && speechRecognition.resumeListening) {
        speechRecognition.resumeListening();
      }
    }, 1200);
    updateVoiceAssistantUI({
      status: voiceSessionActive ? 'Listening...' : 'Ready to talk',
      message: fullReply,
      transcript: 'You said: "' + transcript + '"',
      buttonLabel: voiceSessionActive ? 'Stop' : 'Speak Again',
      listening: false,
      speaking: false,
    });
    if (voiceSessionActive) scheduleVoiceRecognitionRestart();
  }

  // Use XHR for SSE over POST (works on old browsers)
  var xhr = new XMLHttpRequest();
  xhr.open('POST', '/api/chatbot-stream');
  xhr.setRequestHeader('Content-Type', 'application/json');
  var lastIdx = 0;

  function processSSEChunk(text) {
    var lines = text.split('\n');
    for (var i = 0; i < lines.length; i++) {
      var line = lines[i].trim();
      if (!line.startsWith('data: ')) continue;
      var payload = line.slice(6);
      if (payload === '[DONE]') {
        streamDone = true;
        voiceConversationHistory.push({ role: 'user', content: transcript });
        voiceConversationHistory.push({ role: 'assistant', content: fullReply });
        voiceConversationHistory = voiceConversationHistory.slice(-8);
        resetVoiceHistoryTimeout();
        console.log('Robot answer: ' + fullReply);
        if (!isPlaying && audioQueue.length === 0) onAllDone();
        continue;
      }
      try {
        var parsed = JSON.parse(payload);
        if (parsed.error) { console.error('Stream error:', parsed.error); continue; }
        if (parsed.sentence) {
          fullReply += (fullReply ? ' ' : '') + parsed.sentence;
          prefetchTTS(parsed.sentence);
          if (firstSentence) { firstSentence = false; isVoiceSpeaking = true; }
          updateVoiceAssistantUI({
            status: 'Speaking...',
            message: fullReply,
            transcript: 'You said: "' + transcript + '"',
            buttonLabel: voiceSessionActive ? 'Stop' : 'Speak Again',
            listening: false,
            speaking: true,
          });
          playNext();
        }
      } catch(e) {}
    }
  }

  xhr.onprogress = function() {
    var newData = xhr.responseText.substring(lastIdx);
    lastIdx = xhr.responseText.length;
    processSSEChunk(newData);
  };

  xhr.onload = function() {
    if (streamDone) return; // onprogress already handled everything
    // Process the entire response — onprogress may not have fired at all
    processSSEChunk(xhr.responseText);
  };

  xhr.onerror = function() {
    console.error('Chatbot stream failed');
    isVoiceProcessing = false;
    if (speechRecognition && speechRecognition.resumeListening) {
      speechRecognition.resumeListening();
    }
  };

  xhr.send(JSON.stringify({
    message: transcript,
    history: voiceConversationHistory,
  }));
}

function isVoiceAssistantActive() {
  return voiceSessionActive || isVoiceListening || isVoiceSpeaking;
}

function scheduleVoiceRecognitionRestart() {
  if (!voiceSessionActive || manualVoiceStopRequested || isVoiceListening) {
    return;
  }

  // WhisperRecognition manages its own restart loop — skip external restart
  if (speechRecognition && speechRecognition._recording) {
    return;
  }

  clearVoiceRestartTimeout();
  voiceRestartTimeout = setTimeout(() => {
    if (!voiceSessionActive || manualVoiceStopRequested || isVoiceListening || isVoiceSpeaking) {
      return;
    }

    try {
      updateVoiceAssistantUI({
        status: 'Listening...',
        message: 'I am listening. Ask me a library question.',
        transcript: '',
        buttonLabel: 'Stop',
        listening: false,
        speaking: false,
      });
      speechRecognition.start();
    } catch (error) {
      console.warn('Voice recognition restart skipped:', error);
    }
  }, 250);
}

function clearVoiceRestartTimeout() {
  if (voiceRestartTimeout) {
    clearTimeout(voiceRestartTimeout);
    voiceRestartTimeout = null;
  }
}
 
/**
 * Reset inactivity timer (called on any user interaction)
 */
function resetInactivityTimer() {
  // Clear existing timeout
  if (inactivityTimeout) {
    clearTimeout(inactivityTimeout);
  }
  
  // Only set timeout if not on default screen
  if (currentScreen !== 'defaultScreen' && currentScreen !== 'adminScreen') {
    inactivityTimeout = setTimeout(() => {
      console.log('Inactivity timeout - returning to default screen');
      returnToDefault();
    }, INACTIVITY_TIMEOUT);
  }
}
 
/**
 * Reset no-face timer (called when face is detected)
 */
function resetNoFaceTimer() {
  if (noFaceTimeout) {
    clearTimeout(noFaceTimeout);
  }
}
 
/**
 * Start no-face timer (called when face is lost)
 */
function startNoFaceTimer() {
  // Clear any existing timer
  resetNoFaceTimer();
  
  // Only set timeout if not on default screen
  if (currentScreen !== 'defaultScreen' && currentScreen !== 'adminScreen') {
    noFaceTimeout = setTimeout(() => {
      console.log('No face detected for too long - returning to default screen');
      returnToDefault();
    }, NO_FACE_TIMEOUT);
  }
}
 
/**
 * Return to default screen
 */
function returnToDefault() {
  if (currentScreen !== 'defaultScreen') {
    console.log('Returning to default screen');
    
    // If returning from book returns screens, reset scanned count
    if (currentScreen === 'bookReturnsScanScreen' || currentScreen === 'bookReturnsPlaceScreen') {
      bookReturnsManager.state.totalBooksScanned = 0;
      bookReturnsManager.state.scannedBooks = bookReturnsManager.state.scannedBooks.filter(b => b.placed);
      bookReturnsManager.state.isScanning = false;
      bookReturnsManager.state.currentBook = null;
      console.log('Book returns session ended - scanned count reset');
    }

    if (currentScreen === 'selfCheckoutPatronScreen' || currentScreen === 'selfCheckoutScanScreen') {
      if (selfCheckoutManager) {
        selfCheckoutManager.reset();
      }
      updateSelfCheckoutUI();
      console.log('Self checkout session ended - checkout state reset');
    }
    
    switchScreen('defaultScreen');
    previousScreenBeforeAdmin = 'defaultScreen';
    updateVoiceAssistantUI({
      status: 'Waiting to help you...',
      message: 'Tap Speak and ask me about books, directions, hours, printing, or general library help.',
      transcript: 'Listening transcript will appear here.',
      buttonLabel: 'Speak',
      listening: false,
      speaking: false,
    });
    
    // Clear all timers
    if (inactivityTimeout) {
      clearTimeout(inactivityTimeout);
      inactivityTimeout = null;
    }
    if (noFaceTimeout) {
      clearTimeout(noFaceTimeout);
      noFaceTimeout = null;
    }
  }
}
 
/**
 * Screen management functions
 */
function switchScreen(screenId) {
  document.querySelectorAll('.screen').forEach(screen => {
    screen.classList.remove('active');
  });
  document.getElementById(screenId).classList.add('active');
  currentScreen = screenId;
  updateAdminToggleState();
  
  // Reset face detector when returning to default
  if (screenId === 'defaultScreen' && faceDetector) {
    faceDetector.reset();
  }
  
  // Start inactivity timer for non-default screens
  if (screenId !== 'defaultScreen' && screenId !== 'adminScreen') {
    resetInactivityTimer();
  } else if (screenId === 'adminScreen') {
    if (inactivityTimeout) {
      clearTimeout(inactivityTimeout);
      inactivityTimeout = null;
    }
    resetNoFaceTimer();
  }
}
 
function showWelcome() {
  stopVoiceAssistant(true);
  switchScreen('welcomeScreen');
}

function showMenu() {
  stopVoiceAssistant(true);
  switchScreen('menuScreen');
  resetInactivityTimer();
}

function showThankYou() {
  stopVoiceAssistant(true);
  switchScreen('thankYouScreen');
  
  // Clear timers since we're showing thank you
  if (inactivityTimeout) {
    clearTimeout(inactivityTimeout);
    inactivityTimeout = null;
  }
  if (noFaceTimeout) {
    clearTimeout(noFaceTimeout);
    noFaceTimeout = null;
  }
  
  // Return to default after 4 seconds
  setTimeout(() => {
    returnToDefault();
  }, 4000);
}
 
function handleSelection(option) {
  // Reset inactivity timer on interaction
  resetInactivityTimer();
  
  // Route to appropriate feature
  if (option === 'Book Returns') {
    startBookReturns();
  } else if (option === 'Self Checkout') {
    startSelfCheckout();
  } else if (option === 'Book Status') {
    startBookStatus();
  } else if (option === 'Ask Me Anything') {
    // Route to voice assistant on default screen
    switchScreen('defaultScreen');
    toggleVoiceAssistant();
  } else {
    // Other features - implement as needed
    alert(`You selected: ${option}\n\nThis would connect to the ${option.toLowerCase()} system.`);
  }
}
 
/**
 * Book Status Functions
 */
let booksDatabase = [];
let shelfOrderData = null;
 
async function loadBooksDatabase() {
  try {
    const response = await fetch('data/books.json');
    booksDatabase = await response.json();
    console.log(`Books database loaded: ${booksDatabase.length} books`);
  } catch (error) {
    console.error('Failed to load books database:', error);
    booksDatabase = [];
  }
}

async function loadShelfOrderData() {
  if (shelfOrderData) {
    return shelfOrderData;
  }

  const response = await fetch('shelf_viewer/shelf_order.json');
  if (!response.ok) {
    throw new Error(`Failed to load shelf order: ${response.status}`);
  }

  shelfOrderData = await response.json();
  return shelfOrderData;
}

function toggleAdminMode() {
  if (currentScreen === 'adminScreen') {
    const targetScreen = previousScreenBeforeAdmin || 'defaultScreen';
    previousScreenBeforeAdmin = 'defaultScreen';
    switchScreen(targetScreen);
    return;
  }

  previousScreenBeforeAdmin = currentScreen;
  switchScreen('adminScreen');
}

function updateAdminToggleState() {
  const adminToggle = document.getElementById('adminToggle');
  if (!adminToggle) {
    return;
  }

  const isAdmin = currentScreen === 'adminScreen';
  adminToggle.classList.toggle('active', isAdmin);
  adminToggle.textContent = isAdmin ? 'Exit Admin' : 'Admin Mode';
}

function runShelfReading() {
  triggerShelfReading();
}

async function sendStandByPose() {
  await sendNamedPose('Stand-by', STAND_BY_POSE);
}

async function sendHomePose() {
  await sendNamedPose('Home', HOME_POSE);
}

async function sendNamedPose(label, pose) {
  try {
    const response = await fetch(`${ROS_BRIDGE_URL}/standby_pose`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(pose),
    });

    if (!response.ok) {
      throw new Error(`${label} request failed with status ${response.status}`);
    }

    const result = await response.json();
    alert(`${label} pose sent: x=${result.pose.x}, y=${result.pose.y}, theta=${result.pose.theta}`);
  } catch (error) {
    console.error(`Failed to send ${label.toLowerCase()} pose:`, error);
    alert(`Failed to send ${label.toLowerCase()} pose to the ROS bridge.`);
  }
}

async function triggerShelfReading() {
  updateAdminStatus('Starting shelf reading...');

  try {
    const response = await fetch(`${ROS_BRIDGE_URL}/shelf_reading_trigger`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ running: true }),
    });

    if (!response.ok) {
      throw new Error(`Shelf reading trigger failed with status ${response.status}`);
    }

    const result = await response.json();
    if (result.already_running) {
      updateAdminStatus(`Shelf reading already running on ${result.topic}`);
      alert('Shelf reading is already running.');
      return;
    }

    updateAdminStatus(`Shelf reading triggered on ${result.topic}`);
    alert('Shelf reading triggered.');
  } catch (error) {
    console.error('Failed to trigger shelf reading:', error);
    updateAdminStatus('Failed to trigger shelf reading');
    alert('Failed to trigger shelf reading.');
  }
}

function updateAdminStatus(message) {
  const adminStatus = document.getElementById('adminStatus');
  if (adminStatus) {
    adminStatus.textContent = message;
  }
}

async function showShelfOrder() {
  const results = document.getElementById('shelfOrderResults');
  const meta = document.getElementById('shelfOrderMeta');

  if (!results || !meta) {
    return;
  }

  results.className = 'admin-empty';
  results.innerHTML = 'Loading shelf order...';
  meta.textContent = 'Reading shelf_viewer/shelf_order.json';

  try {
    const data = await loadShelfOrderData();
    const sortedBooks = [...(data.books || [])].sort((a, b) => {
      const positionA = Number.isFinite(Number(a.position)) ? Number(a.position) : Number.MAX_SAFE_INTEGER;
      const positionB = Number.isFinite(Number(b.position)) ? Number(b.position) : Number.MAX_SAFE_INTEGER;
      return positionA - positionB;
    });

    if (sortedBooks.length === 0) {
      results.className = 'admin-empty';
      results.innerHTML = 'No books were found in shelf_order.json.';
      meta.textContent = '0 books loaded';
      return;
    }

    results.className = 'shelf-order-list';
    results.innerHTML = sortedBooks.map(createShelfOrderCard).join('');
    meta.textContent = `${sortedBooks.length} books loaded from ${data.image_file || 'shelf snapshot'}`;
  } catch (error) {
    console.error('Failed to load shelf order:', error);
    results.className = 'admin-empty';
    results.innerHTML = 'Unable to load shelf_order.json.';
    meta.textContent = 'Load failed';
  }
}

function createShelfOrderCard(book) {
  const position = Number.isFinite(Number(book.position)) ? Number(book.position) : '?';
  const confidence = typeof book.confidence === 'number'
    ? `${Math.round(book.confidence * 100)}% confidence`
    : 'Confidence unavailable';
  const textComponents = Array.isArray(book.text_components) && book.text_components.length > 0
    ? escapeHtml(book.text_components.join(' | '))
    : 'No OCR tokens available';

  return `
    <div class="shelf-order-card">
      <div class="shelf-order-position">${position}</div>
      <div>
        <div class="shelf-order-call">${escapeHtml(book.call_number || 'Unknown call number')}</div>
        <div class="shelf-order-detail">OCR tokens: ${textComponents}</div>
      </div>
      <div class="shelf-order-confidence">${confidence}</div>
    </div>
  `;
}

function escapeHtml(value) {
  return String(value)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#39;');
}

function startBookStatus() {
  switchScreen('bookStatusScreen');
  resetInactivityTimer();
  
  // Clear previous search
  document.getElementById('bookSearchInput').value = '';
  showSearchPlaceholder();
  
  // Focus on search input
  setTimeout(() => {
    document.getElementById('bookSearchInput').focus();
  }, 300);
}
 
function searchBooks() {
  const query = document.getElementById('bookSearchInput').value.trim().toLowerCase();
  resetInactivityTimer();
  
  if (!query) {
    showSearchPlaceholder();
    return;
  }
  
  // Search through books
  const results = booksDatabase.filter(book => {
    return (
      book.title.toLowerCase().includes(query) ||
      book.author.toLowerCase().includes(query) ||
      book.isbn.includes(query) ||
      book.category.toLowerCase().includes(query)
    );
  });
  
  displaySearchResults(results, query);
}
 
function showSearchPlaceholder() {
  const resultsDiv = document.getElementById('searchResults');
  resultsDiv.innerHTML = `
    <div class="search-placeholder">
      <div class="placeholder-icon">📚</div>
      <p class="placeholder-text">Enter a book title, author, or ISBN to search</p>
    </div>
  `;
}
 
function displaySearchResults(results, query) {
  const resultsDiv = document.getElementById('searchResults');
  
  if (results.length === 0) {
    resultsDiv.innerHTML = `
      <div class="no-results">
        <div class="no-results-icon">😞</div>
        <p class="no-results-text">No books found for "${query}"</p>
        <p style="margin-top: 10px; color: #999;">Try a different search term</p>
      </div>
    `;
    return;
  }
  
  resultsDiv.innerHTML = results.map(book => createBookCard(book)).join('');
}
 
function createBookCard(book) {
  const statusClass = `status-${book.status.replace('-', '-')}`;
  const statusText = book.status === 'checked-out' ? 'Checked Out' : 
                     book.status === 'on-hold' ? 'On Hold' : 
                     'Available';
  
  let statusInfo = '';
  if (book.status === 'checked-out' && book.dueDate) {
    statusInfo = `<div class="detail-item">
      <span class="detail-label">Due Date:</span>
      <span class="detail-value due-date">${formatDate(book.dueDate)}</span>
    </div>`;
  }
  
  if (book.status === 'on-hold' && book.holds) {
    statusInfo = `<div class="detail-item">
      <span class="detail-label">Holds:</span>
      <span class="detail-value">${book.holds} people waiting</span>
    </div>`;
  }
  
  return `
    <div class="book-card">
      <div class="book-card-header">
        <div class="book-info">
          <div class="book-title">${book.title}</div>
          <div class="book-author">by ${book.author}</div>
          <div class="book-meta">ISBN: ${book.isbn} • ${book.category}</div>
        </div>
        <div class="book-status-badge ${statusClass}">
          ${statusText}
        </div>
      </div>
      <div class="book-details">
        <div class="detail-item">
          <span class="detail-label">Location:</span>
          <span class="detail-value">${book.location}</span>
        </div>
        <div class="detail-item">
          <span class="detail-label">Call Number:</span>
          <span class="detail-value">${book.callNumber}</span>
        </div>
        ${statusInfo}
      </div>
    </div>
  `;
}
 
function formatDate(dateString) {
  const parts = String(dateString).split('-').map(Number);
  const date = parts.length === 3 && parts.every(Number.isFinite)
    ? new Date(parts[0], parts[1] - 1, parts[2])
    : new Date(dateString);
  const options = { month: 'short', day: 'numeric', year: 'numeric' };
  return date.toLocaleDateString('en-US', options);
}
 
function exitBookStatus() {
  returnToDefault();
}
 
// Add enter key support for search
document.addEventListener('DOMContentLoaded', () => {
  setTimeout(() => {
    const searchInput = document.getElementById('bookSearchInput');
    if (searchInput) {
      searchInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') {
          searchBooks();
        }
      });
    }

    const checkoutNameInput = document.getElementById('checkoutNameInput');
    const checkoutIdInput = document.getElementById('checkoutIdInput');
    const checkoutScanInput = document.getElementById('checkoutScanInput');

    [checkoutNameInput, checkoutIdInput].forEach((input) => {
      if (input) {
        input.addEventListener('keypress', (e) => {
          if (e.key === 'Enter') {
            beginSelfCheckout();
          }
        });
      }
    });

    if (checkoutScanInput) {
      checkoutScanInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') {
          handleSelfCheckoutScan();
        }
      });
    }
  }, 1000);
});

/**
 * Mic-reactive voice visualizer
 * - Listening: drives bars from real microphone frequency data via Web Audio API
 * - Speaking: organic sine-wave simulation so each bar moves independently
 */
const VoiceVisualizer = (() => {
  // Pure CSS animation — JS just toggles state, zero per-frame work
  return {
    startListening() {},
    startSpeaking() {},
    stop() {},
  };
})();

function stopVoiceAssistant(clearHistory) {
  pendingVoiceResponse = '';
  manualVoiceStopRequested = false;
  voiceSessionActive = false;
  if (clearHistory) { clearVoiceHistory(); } else { resetVoiceHistoryTimeout(); }
  clearVoiceRestartTimeout();

  if (speechRecognition && isVoiceListening) {
    manualVoiceStopRequested = true;
    speechRecognition.stop();
  }

  if (typeof _ttsAudio !== 'undefined' && _ttsAudio) { _ttsAudio.pause(); _ttsAudio = null; }
  if (window.speechSynthesis) {
    window.speechSynthesis.cancel();
  }

  isVoiceListening = false;
  isVoiceSpeaking = false;

  updateVoiceAssistantUI({
    status: currentScreen === 'defaultScreen' ? 'Waiting to help you...' : 'Ready',
    message: 'Tap Speak and ask me about books, directions, hours, printing, or general library help.',
    transcript: 'Listening transcript will appear here.',
    buttonLabel: speechRecognitionSupported ? 'Speak' : 'Voice Unavailable',
    listening: false,
    speaking: false,
  });
}
 
/**
 * Utility functions
 */
function playWelcomeSound() {
  // Implement audio playback here
  // Example:
  // const audio = new Audio('sounds/welcome.mp3');
  // audio.play();
  console.log('Playing welcome sound...');
}
 
/**
 * Initialize on page load
 */
window.addEventListener('load', () => {
  init();
  updateAdminToggleState();
  
  // Track user interactions to reset inactivity timer
  document.addEventListener('click', resetInactivityTimer);
  document.addEventListener('touchstart', resetInactivityTimer);
  document.addEventListener('mousemove', throttle(resetInactivityTimer, 1000));
  document.addEventListener('keypress', resetInactivityTimer);
});
 
/**
 * Throttle function to limit how often a function is called
 */
function throttle(func, wait) {
  let timeout;
  return function executedFunction(...args) {
    const later = () => {
      clearTimeout(timeout);
      func(...args);
    };
    clearTimeout(timeout);
    timeout = setTimeout(later, wait);
  };
}
 
/**
 * Cleanup on page unload
 */
window.addEventListener('beforeunload', () => {
  stopVoiceAssistant();

  if (faceDetector) {
    faceDetector.stop();
  }
  
  if (video && video.srcObject) {
    video.srcObject.getTracks().forEach(track => track.stop());
  }
  
  // Clear all timers
  if (inactivityTimeout) {
    clearTimeout(inactivityTimeout);
  }
  if (noFaceTimeout) {
    clearTimeout(noFaceTimeout);
  }
});
