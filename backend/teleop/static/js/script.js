// Make sure this is at the top of your script.js file, outside any functions
let isFullscreen = false;

// Then add a simple debug function to check its state
function checkFullscreenState() {
    console.log("Current fullscreen state:", isFullscreen);
}

function enterFullscreen() {
    console.log("Entering fullscreen mode");
    isFullscreen = true;
    
    // Get all the elements we need to modify
    const body = document.body;
    const mainContent = document.querySelector('.main-content');
    const container = document.querySelector('.container');
    const videoColumn = document.querySelector('.video-column');
    const videoFeeds = document.querySelector('.video-feeds');
    const primaryFeed = document.querySelector('.video-feed.primary');
    const primaryFeedContainer = document.querySelector('.video-feed.primary .feed-container');
    const primaryFeedImg = document.querySelector('.video-feed.primary .feed-container img');
    const primaryFeedOverlay = document.querySelector('.video-feed.primary .feed-overlay');
    const feedLabel = primaryFeedOverlay ? primaryFeedOverlay.querySelector('.feed-label') : null;
    const feedControls = primaryFeedOverlay ? primaryFeedOverlay.querySelector('.feed-controls') : null;
    const secondaryFeed = document.querySelector('.video-feed.secondary');
    const controlsColumn = document.querySelector('.controls-column');
    const statusHistoryColumn = document.querySelector('.status-history-column');
    const fullscreenBtn = document.getElementById('frontCamFullscreen');
    
    // Get the original background color
    const originalBgColor = getComputedStyle(body).backgroundColor || '#1e272e';
    
    if (!mainContent) {
        console.error("Main content element not found");
        return;
    }
    
    // Prevent scrolling on body
    body.style.overflow = 'hidden';
    
    // Add fullscreen class to main content
    mainContent.classList.add('fullscreen-mode');
    
    // Change main content to take up the full viewport
    mainContent.style.position = 'fixed';
    mainContent.style.top = '0';
    mainContent.style.left = '0';
    mainContent.style.width = '100vw';
    mainContent.style.height = '100vh';
    mainContent.style.padding = '0';
    mainContent.style.margin = '0';
    mainContent.style.backgroundColor = originalBgColor;
    mainContent.style.display = 'flex';
    mainContent.style.justifyContent = 'center';
    mainContent.style.alignItems = 'center';
    mainContent.style.zIndex = '9999';
    
    // Reset all positioning and layout styles
    if (container) {
        container.style.padding = "0";
        container.style.margin = "0";
        container.style.maxWidth = "100%";
        container.style.width = "100%";
        container.style.height = "100%";
        container.style.overflow = "hidden";
    }
    
    // Position video column to fill the entire screen
    if (videoColumn) {
        videoColumn.style.position = "fixed";
        videoColumn.style.top = "0";
        videoColumn.style.left = "0";
        videoColumn.style.width = "100vw";
        videoColumn.style.height = "100vh";
        videoColumn.style.display = "flex";
        videoColumn.style.justifyContent = "center";
        videoColumn.style.alignItems = "center";
        videoColumn.style.padding = "0";
        videoColumn.style.margin = "0";
        videoColumn.style.zIndex = "1000";
    }
    
    // Make video feeds container fill the entire screen
    if (videoFeeds) {
        videoFeeds.style.position = "fixed";
        videoFeeds.style.top = "0";
        videoFeeds.style.left = "0";
        videoFeeds.style.width = "100vw";
        videoFeeds.style.height = "100vh";
        videoFeeds.style.display = "flex";
        videoFeeds.style.justifyContent = "center";
        videoFeeds.style.alignItems = "center";
        videoFeeds.style.padding = "0";
        videoFeeds.style.margin = "0";
    }
    
    // Position primary feed to fill the entire screen
    if (primaryFeed) {
        primaryFeed.style.position = "fixed";
        primaryFeed.style.top = "0";
        primaryFeed.style.left = "0";
        primaryFeed.style.width = "100vw";
        primaryFeed.style.height = "100vh";
        primaryFeed.style.display = "flex";
        primaryFeed.style.justifyContent = "center";
        primaryFeed.style.alignItems = "center";
        primaryFeed.style.margin = "0";
        primaryFeed.style.zIndex = "1000";
    }
    
    // Make primary feed container fill the entire screen
    if (primaryFeedContainer) {
        primaryFeedContainer.style.position = "fixed";
        primaryFeedContainer.style.top = "0";
        primaryFeedContainer.style.left = "0";
        primaryFeedContainer.style.width = "100vw";
        primaryFeedContainer.style.height = "100vh";
        primaryFeedContainer.style.maxHeight = "none";
        primaryFeedContainer.style.maxWidth = "none";
        primaryFeedContainer.style.minWidth = "100vw";
        primaryFeedContainer.style.minHeight = "100vh";
        primaryFeedContainer.style.borderRadius = "0";
        primaryFeedContainer.style.border = "none";
        primaryFeedContainer.style.margin = "0";
        primaryFeedContainer.style.overflow = "hidden";
        primaryFeedContainer.style.backgroundColor = "black";
        primaryFeedContainer.style.display = "flex";
        primaryFeedContainer.style.justifyContent = "center";
        primaryFeedContainer.style.alignItems = "center";
    }
    
    // Ensure the image fills the entire screen
    if (primaryFeedImg) {
        primaryFeedImg.style.width = "100%";
        primaryFeedImg.style.height = "100%";
        primaryFeedImg.style.objectFit = "cover"; // Changed to cover to fill the screen
        primaryFeedImg.style.position = "absolute";
    }
    
    // Make sure the overlay stays aligned with the feed container
    if (primaryFeedOverlay) {
        primaryFeedOverlay.style.position = "fixed";
        primaryFeedOverlay.style.top = "0";
        primaryFeedOverlay.style.left = "0";
        primaryFeedOverlay.style.right = "0";
        primaryFeedOverlay.style.width = "100%";
        primaryFeedOverlay.style.boxSizing = "border-box";
        primaryFeedOverlay.style.zIndex = "1001";
        primaryFeedOverlay.style.padding = "1rem";
        primaryFeedOverlay.style.pointerEvents = "auto";
        primaryFeedOverlay.style.background = "linear-gradient(180deg, rgba(0,0,0,0.6) 0%, transparent 100%)";
        primaryFeedOverlay.style.display = "flex";
        primaryFeedOverlay.style.justifyContent = "space-between";
        primaryFeedOverlay.style.alignItems = "center";
    }
    
    // Ensure the feed label is visible
    if (feedLabel) {
        feedLabel.style.color = "var(--text-primary)";
        feedLabel.style.fontWeight = "600";
        feedLabel.style.fontSize = "1.2rem"; // Increased size for better visibility
        feedLabel.style.textTransform = "uppercase";
        feedLabel.style.letterSpacing = "0.05em";
        feedLabel.style.textShadow = "0 2px 4px rgba(0,0,0,0.5)";
    }
    
    // Ensure the feed controls are properly aligned
    if (feedControls) {
        feedControls.style.display = "flex";
        feedControls.style.alignItems = "center";
        feedControls.style.gap = "0.5rem";
    }
    
    // Position secondary feed at the top center
    if (secondaryFeed) {
        secondaryFeed.style.position = "fixed";
        secondaryFeed.style.top = "20px";
        secondaryFeed.style.left = "50%";
        secondaryFeed.style.transform = "translateX(-50%)";
        secondaryFeed.style.width = "300px";
        secondaryFeed.style.zIndex = "1002";
    }
    
    // Position controls column on the right
    if (controlsColumn) {
        controlsColumn.style.position = "fixed";
        controlsColumn.style.right = "20px";
        controlsColumn.style.top = "50%";
        controlsColumn.style.transform = "translateY(-50%)";
        controlsColumn.style.width = "300px";
        controlsColumn.style.maxHeight = "90vh";
        controlsColumn.style.overflowY = "auto";
        controlsColumn.style.margin = "0";
        controlsColumn.style.opacity = "0.85";
        controlsColumn.style.zIndex = "1002";
    }
    
    // Position status history column on the left
    if (statusHistoryColumn) {
        statusHistoryColumn.style.position = "fixed";
        statusHistoryColumn.style.left = "20px";
        statusHistoryColumn.style.top = "50%";
        statusHistoryColumn.style.transform = "translateY(-50%)";
        statusHistoryColumn.style.width = "300px";
        statusHistoryColumn.style.maxHeight = "90vh";
        statusHistoryColumn.style.overflowY = "auto";
        statusHistoryColumn.style.margin = "0";
        statusHistoryColumn.style.opacity = "0.85";
        statusHistoryColumn.style.zIndex = "1002";
    }
    
    // Update fullscreen button to show exit icon
    if (fullscreenBtn) {
        fullscreenBtn.textContent = "⤓"; // Change to exit fullscreen icon
        fullscreenBtn.title = "Exit Fullscreen";
        fullscreenBtn.style.zIndex = "1003"; // Ensure it's above other elements
        fullscreenBtn.style.fontSize = "1.2rem"; // Increased size for better visibility
    }
    
    // Add a small instruction about Escape key
    const escapeHint = document.createElement('div');
    escapeHint.className = 'escape-hint';
    escapeHint.textContent = 'Press ESC to exit fullscreen';
    escapeHint.style.position = 'fixed';
    escapeHint.style.bottom = '20px';
    escapeHint.style.right = '20px';
    escapeHint.style.background = 'rgba(0, 0, 0, 0.7)';
    escapeHint.style.color = 'white';
    escapeHint.style.padding = '8px 12px';
    escapeHint.style.borderRadius = '4px';
    escapeHint.style.fontSize = '14px';
    escapeHint.style.zIndex = '1003';
    escapeHint.style.opacity = '0.8';
    document.body.appendChild(escapeHint);
    
    // Hide the header
    const header = document.querySelector('.header');
    if (header) {
        header.style.display = 'none';
    }
    
    console.log("Fullscreen mode enabled");
}

function exitFullscreen() {
    console.log("Exiting fullscreen mode");
    isFullscreen = false;
    
    // Get all the elements we need to modify
    const body = document.body;
    const mainContent = document.querySelector('.main-content');
    const container = document.querySelector('.container');
    const videoColumn = document.querySelector('.video-column');
    const videoFeeds = document.querySelector('.video-feeds');
    const primaryFeed = document.querySelector('.video-feed.primary');
    const primaryFeedContainer = document.querySelector('.video-feed.primary .feed-container');
    const primaryFeedImg = document.querySelector('.video-feed.primary .feed-container img');
    const primaryFeedOverlay = document.querySelector('.video-feed.primary .feed-overlay');
    const feedLabel = primaryFeedOverlay ? primaryFeedOverlay.querySelector('.feed-label') : null;
    const feedControls = primaryFeedOverlay ? primaryFeedOverlay.querySelector('.feed-controls') : null;
    const secondaryFeed = document.querySelector('.video-feed.secondary');
    const controlsColumn = document.querySelector('.controls-column');
    const statusHistoryColumn = document.querySelector('.status-history-column');
    const fullscreenBtn = document.getElementById('frontCamFullscreen');
    
    if (!mainContent) {
        console.error("Main content element not found");
        return;
    }
    
    // Restore body scrolling
    body.style.overflow = '';
    
    // Remove fullscreen class
    mainContent.classList.remove('fullscreen-mode');
    
    // Reset main content styles
    mainContent.style.display = '';
    mainContent.style.flexDirection = '';
    mainContent.style.justifyContent = '';
    mainContent.style.alignItems = '';
    mainContent.style.width = '';
    mainContent.style.height = '';
    mainContent.style.padding = '';
    mainContent.style.margin = '';
    mainContent.style.backgroundColor = '';
    
    // Reset container styles
    if (container) {
        container.style.padding = "";
        container.style.margin = "";
        container.style.maxWidth = "";
        container.style.width = "";
        container.style.height = "";
        container.style.overflow = "";
    }
    
    // Reset video column styles
    if (videoColumn) {
        videoColumn.style.position = "";
        videoColumn.style.top = "";
        videoColumn.style.left = "";
        videoColumn.style.right = "";
        videoColumn.style.width = "";
        videoColumn.style.height = "";
        videoColumn.style.display = "";
        videoColumn.style.justifyContent = "";
        videoColumn.style.alignItems = "";
        videoColumn.style.padding = "";
        videoColumn.style.margin = "";
        videoColumn.style.flex = "";
        videoColumn.style.order = "";
    }
    
    // Reset video feeds styles
    if (videoFeeds) {
        videoFeeds.style.position = "";
        videoFeeds.style.width = "";
        videoFeeds.style.height = "";
        videoFeeds.style.display = "";
        videoFeeds.style.flexDirection = "";
        videoFeeds.style.justifyContent = "";
        videoFeeds.style.alignItems = "";
        videoFeeds.style.padding = "";
        videoFeeds.style.margin = "";
    }
    
    // Reset primary feed styles
    if (primaryFeed) {
        primaryFeed.style.position = "";
        primaryFeed.style.width = "";
        primaryFeed.style.height = "";
        primaryFeed.style.display = "";
        primaryFeed.style.justifyContent = "";
        primaryFeed.style.alignItems = "";
        primaryFeed.style.margin = "";
    }
    
    // Reset primary feed container styles
    if (primaryFeedContainer) {
        primaryFeedContainer.style.position = "";
        primaryFeedContainer.style.width = "";
        primaryFeedContainer.style.height = "";
        primaryFeedContainer.style.maxHeight = "";
        primaryFeedContainer.style.maxWidth = "";
        primaryFeedContainer.style.minWidth = "";
        primaryFeedContainer.style.minHeight = "";
        primaryFeedContainer.style.borderRadius = "";
        primaryFeedContainer.style.border = "";
        primaryFeedContainer.style.margin = "";
        primaryFeedContainer.style.overflow = "";
        primaryFeedContainer.style.backgroundColor = "";
        primaryFeedContainer.style.display = "";
        primaryFeedContainer.style.justifyContent = "";
        primaryFeedContainer.style.alignItems = "";
    }
    
    // Reset primary feed image styles
    if (primaryFeedImg) {
        primaryFeedImg.style.width = "";
        primaryFeedImg.style.height = "";
        primaryFeedImg.style.objectFit = "";
        primaryFeedImg.style.position = "";
    }
    
    // Reset primary feed overlay styles
    if (primaryFeedOverlay) {
        primaryFeedOverlay.style.position = "";
        primaryFeedOverlay.style.top = "";
        primaryFeedOverlay.style.left = "";
        primaryFeedOverlay.style.right = "";
        primaryFeedOverlay.style.width = "";
        primaryFeedOverlay.style.boxSizing = "";
        primaryFeedOverlay.style.zIndex = "";
        primaryFeedOverlay.style.padding = "";
        primaryFeedOverlay.style.pointerEvents = "";
        primaryFeedOverlay.style.background = "";
        primaryFeedOverlay.style.display = "";
        primaryFeedOverlay.style.justifyContent = "";
        primaryFeedOverlay.style.alignItems = "";
    }
    
    // Reset feed label styles
    if (feedLabel) {
        feedLabel.style.color = "";
        feedLabel.style.fontWeight = "";
        feedLabel.style.fontSize = "";
        feedLabel.style.textTransform = "";
        feedLabel.style.letterSpacing = "";
        feedLabel.style.textShadow = "";
    }
    
    // Reset feed controls styles
    if (feedControls) {
        feedControls.style.display = "";
        feedControls.style.alignItems = "";
        feedControls.style.gap = "";
    }
    
    // Reset secondary feed styles
    if (secondaryFeed) {
        secondaryFeed.style.position = "";
        secondaryFeed.style.top = "";
        secondaryFeed.style.left = "";
        secondaryFeed.style.transform = "";
        secondaryFeed.style.width = "";
        secondaryFeed.style.zIndex = "";
    }
    
    // Reset controls column styles
    if (controlsColumn) {
        controlsColumn.style.position = "";
        controlsColumn.style.width = "";
        controlsColumn.style.maxHeight = "";
        controlsColumn.style.overflowY = "";
        controlsColumn.style.margin = "";
        controlsColumn.style.opacity = "";
        controlsColumn.style.zIndex = "";
        controlsColumn.style.order = "";
    }
    
    // Reset status history column styles
    if (statusHistoryColumn) {
        statusHistoryColumn.style.position = "";
        statusHistoryColumn.style.width = "";
        statusHistoryColumn.style.maxHeight = "";
        statusHistoryColumn.style.overflowY = "";
        statusHistoryColumn.style.margin = "";
        statusHistoryColumn.style.opacity = "";
        statusHistoryColumn.style.zIndex = "";
        statusHistoryColumn.style.order = "";
    }
    
    // Reset fullscreen button to show fullscreen icon
    if (fullscreenBtn) {
        fullscreenBtn.textContent = "⛶"; // Change back to fullscreen icon
        fullscreenBtn.title = "Fullscreen";
        fullscreenBtn.style.zIndex = "";
    }
    
    // Remove escape hint
    const escapeHint = document.querySelector('.escape-hint');
    if (escapeHint) {
        escapeHint.remove();
    }
    
    // Show the header again
    const header = document.querySelector('.header');
    if (header) {
        header.style.display = '';
    }
    
    console.log("Fullscreen mode disabled");
}

function toggleFullscreen() {
    console.log("toggleFullscreen called, current state:", isFullscreen);
    if (!isFullscreen) {
        enterFullscreen();
    } else {
        exitFullscreen();
    }
}

// Set up the fullscreen button and escape key handler
(function() {
    // Function to set up the fullscreen button
    function setupFullscreenButton() {
        const fullscreenBtn = document.getElementById('frontCamFullscreen');
        if (!fullscreenBtn) {
            console.error("Fullscreen button not found in the DOM");
            return;
        }
        
        console.log("Fullscreen button found, setting up direct click handler");
        
        // Remove the button and replace it with a clone to remove any existing listeners
        const parent = fullscreenBtn.parentNode;
        const newBtn = fullscreenBtn.cloneNode(true);
        parent.replaceChild(newBtn, fullscreenBtn);
        
        // Add a direct onclick handler
        newBtn.onclick = function(e) {
            e.preventDefault();
            e.stopPropagation();
            console.log("Fullscreen button clicked directly");
            
            // Direct call to enterFullscreen
            if (!isFullscreen) {
                enterFullscreen();
            } else {
                exitFullscreen();
            }
            
            return false;
        };
    }
    
    // Set up the escape key handler
    function setupEscapeKeyHandler() {
        document.addEventListener('keydown', function(e) {
            if (e.key === 'Escape' && isFullscreen) {
                console.log("Escape key pressed, exiting fullscreen");
                exitFullscreen();
            }
        });
    }
    
    // Try to set up the button immediately if the DOM is already loaded
    if (document.readyState === "complete" || document.readyState === "interactive") {
        setupFullscreenButton();
        setupEscapeKeyHandler();
    }
    
    // Also set it up on DOMContentLoaded to be safe
    document.addEventListener('DOMContentLoaded', function() {
        setupFullscreenButton();
        setupEscapeKeyHandler();
    });
    
    // And set it up on load as a final fallback
    window.addEventListener('load', function() {
        setupFullscreenButton();
        setupEscapeKeyHandler();
    });
})();
