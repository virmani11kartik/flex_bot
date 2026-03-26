/**
 * Library Robot Interface - Main Application
 * Integrates face detection with UI screens
 */
 
// ============================================================================
// CONFIGURATION - Adjust these values to customize behavior
// ============================================================================
 
// Inactivity Timeouts (in milliseconds)
const INACTIVITY_TIMEOUT = 30000; // 30 seconds - Return to home after no clicks/touches
const NO_FACE_TIMEOUT = 15000;    // 15 seconds - Return to home when face not detected
 
// Face Detection Settings
const FACE_DETECTION_CONFIG = {
  minDetectionConfidence: 0.8,    // Minimum confidence for face detection (0-1)
  gazeThreshold: 0.6,             // How centered the face needs to be (0-1, higher = stricter)
  requiredGazeDuration: 2000,     // Time in ms person must look before engaging (2 seconds)
  faceSizeThreshold: 0.12,        // Minimum face size to prevent distant detections (0-1)
  detectionInterval: 100,         // Time between detection checks in ms
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
 
// Book Returns
let bookReturnsManager = null;
 
// DOM Elements
const elements = {
  video: null,
  overlay: null,
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
  
  video = elements.video;
  overlayCanvas = elements.overlay;
  overlayContext = overlayCanvas.getContext('2d');
  
  // Initialize camera
  await setupCamera();
  
  // Initialize face detection
  await initializeFaceDetection();
  
  // Initialize book returns manager
  initializeBookReturns();
  
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
      gazeThreshold: config.gazeThreshold || 0.7,
      requiredGazeDuration: config.requiredGazeDuration || 2000,
      faceSizeThreshold: config.faceSizeThreshold || 0.15,
      detectionInterval: config.detectionInterval || 100,
    };
    
    // State
    this.state = {
      isLookingAtScreen: false,
      gazeStartTime: null,
      currentFace: null,
      faceHistory: [],
      historySize: 5,
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
  }
  
  async detect() {
    if (!this.isRunning) return;
    
    try {
      // Get face predictions
      const predictions = await this.model.estimateFaces(this.video, false);
      
      // Process predictions
      this.processPredictions(predictions);
      
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
  
  processPredictions(predictions) {
    const now = Date.now();
    
    if (predictions.length === 0) {
      this.handleNoFace();
      return;
    }
    
    // Reset no-face timer since we detected a face
    if (typeof resetNoFaceTimer === 'function') {
      resetNoFaceTimer();
    }
    
    // Get primary face (largest)
    const primaryFace = this.getPrimaryFace(predictions);
    if (!primaryFace) {
      this.handleNoFace();
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
    
    // Check if looking at screen
    const isLooking = this.isLookingAtScreen(metrics);
    
    if (isLooking) {
      this.handleLooking(now);
    } else {
      this.handleNotLooking();
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
  
  isLookingAtScreen(metrics) {
    // Check confidence
    if (metrics.confidence < this.config.minDetectionConfidence) {
      return false;
    }
    
    // Check size (person close enough)
    if (metrics.size < this.config.faceSizeThreshold) {
      return false;
    }
    
    // Check if centered
    const horizontalCenter = Math.abs(metrics.centerX - 0.5);
    const verticalCenter = Math.abs(metrics.centerY - 0.5);
    
    const threshold = (1 - this.config.gazeThreshold) / 2;
    const isCentered = horizontalCenter < threshold && verticalCenter < threshold;
    
    return isCentered;
  }
  
  handleLooking(now) {
    if (!this.state.isLookingAtScreen) {
      // Started looking
      this.state.isLookingAtScreen = true;
      this.state.gazeStartTime = now;
      
      if (this.onGazeStart) {
        this.onGazeStart();
      }
    } else {
      // Check duration
      const duration = now - this.state.gazeStartTime;
      
      if (duration >= this.config.requiredGazeDuration && !this.hasTriggeredWelcome) {
        this.hasTriggeredWelcome = true;
        
        if (this.onEngaged) {
          this.onEngaged({ duration, timestamp: now });
        }
      }
    }
  }
  
  handleNotLooking() {
    if (this.state.isLookingAtScreen) {
      this.state.isLookingAtScreen = false;
      this.state.gazeStartTime = null;
      
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
  if (currentScreen === 'defaultScreen') {
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
    
    switchScreen('defaultScreen');
    previousScreenBeforeAdmin = 'defaultScreen';
    
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
  switchScreen('welcomeScreen');
}
 
function showMenu() {
  switchScreen('menuScreen');
  resetInactivityTimer();
}
 
function showThankYou() {
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
  } else if (option === 'Book Status') {
    startBookStatus();
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
  const date = new Date(dateString);
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
  }, 1000);
});
 
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
