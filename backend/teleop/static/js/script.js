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
    mainContent.style.justifyContent = 'space-between'; // Changed to space-between for 3-column layout
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
    
    // Define side panel width and spacing
    const sideColumnWidth = 400; // Keep the width the same
    const sideColumnMargin = 10; // Reduced from 30px to 10px to bring containers closer
    const totalSideWidth = sideColumnWidth + sideColumnMargin;
    
    // Position status history column on the left
    if (statusHistoryColumn) {
        statusHistoryColumn.style.position = "fixed";
        statusHistoryColumn.style.left = `${sideColumnMargin}px`;
        statusHistoryColumn.style.top = "50%";
        statusHistoryColumn.style.transform = "translateY(-50%)";
        statusHistoryColumn.style.width = `${sideColumnWidth}px`;
        statusHistoryColumn.style.maxHeight = "95vh"; // Increased from 90vh to use more vertical space
        statusHistoryColumn.style.overflowY = "auto";
        statusHistoryColumn.style.margin = "0";
        statusHistoryColumn.style.opacity = "1";
        statusHistoryColumn.style.zIndex = "1002";
        statusHistoryColumn.style.padding = "10px";
        statusHistoryColumn.style.boxSizing = "border-box";
        
        // Ensure all panels within the status history column have proper width
        const statusPanels = statusHistoryColumn.querySelectorAll('.status-panel, .history-panel');
        statusPanels.forEach(panel => {
            panel.style.width = "100%";
            panel.style.boxSizing = "border-box";
            panel.style.overflow = "visible";
            panel.style.margin = "0 0 20px 0"; // Add bottom margin between panels
            panel.style.padding = "15px"; // Increase padding inside panels
        });
        
        // Specifically target the vehicle status panel to make it taller
        const vehicleStatusPanel = statusHistoryColumn.querySelector('.status-panel');
        if (vehicleStatusPanel) {
            vehicleStatusPanel.style.minHeight = "520px"; // Increased from default to accommodate all status boxes
            vehicleStatusPanel.style.height = "auto"; // Allow it to grow if needed
        }
        
        // Ensure status grid items have proper width and don't get cut off
        const statusGrid = statusHistoryColumn.querySelector('.status-grid');
        if (statusGrid) {
            statusGrid.style.gridTemplateColumns = "1fr 1fr"; // Ensure 2 columns of equal width
            statusGrid.style.width = "100%";
            statusGrid.style.boxSizing = "border-box";
            statusGrid.style.gap = "15px"; // Increase gap between grid items
            statusGrid.style.padding = "5px"; // Add some padding around the grid
            
            // Make sure each status item has enough space
            const statusItems = statusGrid.querySelectorAll('.status-item');
            statusItems.forEach(item => {
                item.style.width = "100%";
                item.style.boxSizing = "border-box";
                item.style.minWidth = "0"; // Allow items to shrink if needed
                item.style.overflow = "hidden"; // Hide overflow if necessary
                item.style.display = "flex";
                item.style.alignItems = "center";
                item.style.padding = "12px"; // Increase padding inside items
                item.style.marginBottom = "10px"; // Add bottom margin to each item
                
                // Ensure status values don't get cut off
                const statusValue = item.querySelector('.status-value');
                if (statusValue) {
                    statusValue.style.whiteSpace = "nowrap";
                    statusValue.style.overflow = "hidden";
                    statusValue.style.textOverflow = "ellipsis";
                    statusValue.style.fontSize = "1.1rem"; // Slightly larger font
                }
                
                // Ensure status icons have enough space
                const statusIcon = item.querySelector('.status-icon');
                if (statusIcon) {
                    statusIcon.style.marginRight = "10px"; // Add space between icon and text
                    statusIcon.style.fontSize = "1.5rem"; // Slightly larger icon
                }
            });
        }
        
        // Ensure command log has proper width
        const commandLog = statusHistoryColumn.querySelector('.command-log');
        if (commandLog) {
            commandLog.style.width = "100%";
            commandLog.style.boxSizing = "border-box";
            commandLog.style.height = "300px"; // Fixed height for command log
            commandLog.style.padding = "10px"; // Add padding inside command log
        }
    }
    
    // Position controls column on the right
    if (controlsColumn) {
        controlsColumn.style.position = "fixed";
        controlsColumn.style.right = `${sideColumnMargin}px`; // Reduced margin
        controlsColumn.style.top = "50%";
        controlsColumn.style.transform = "translateY(-50%)";
        controlsColumn.style.width = `${sideColumnWidth}px`;
        controlsColumn.style.maxHeight = "95vh"; // Increased from 90vh to use more vertical space
        controlsColumn.style.overflowY = "auto";
        controlsColumn.style.margin = "0";
        controlsColumn.style.opacity = "1";
        controlsColumn.style.zIndex = "1002";
        controlsColumn.style.padding = "10px";
        controlsColumn.style.boxSizing = "border-box";
        
        // Ensure all panels within the controls column have proper width
        const controlPanels = controlsColumn.querySelectorAll('.control-panel');
        controlPanels.forEach(panel => {
            panel.style.width = "100%";
            panel.style.boxSizing = "border-box";
            panel.style.overflow = "visible";
            panel.style.margin = "0 0 20px 0"; // Add bottom margin between panels
            panel.style.padding = "15px"; // Increase padding inside panels
        });
        
        // Ensure keyboard controls have enough space and proper layout
        const keyboardControls = controlsColumn.querySelector('.keyboard-controls');
        if (keyboardControls) {
            // Ensure the controls content has proper layout
            const controlsContent = keyboardControls.querySelector('.controls-content');
            if (controlsContent) {
                controlsContent.style.width = "100%";
                controlsContent.style.padding = "0";
                controlsContent.style.boxSizing = "border-box";
            }
            
            // Ensure the keyboard controls wrapper has proper layout
            const keyboardWrapper = keyboardControls.querySelector('.keyboard-controls-wrapper');
            if (keyboardWrapper) {
                keyboardWrapper.style.width = "100%";
                keyboardWrapper.style.boxSizing = "border-box";
            }
            
            // Fix key rows layout
            const keyRows = keyboardControls.querySelectorAll('.key-row');
            keyRows.forEach(row => {
                row.style.marginBottom = "15px"; // Add space between rows
                row.style.display = "flex";
                row.style.justifyContent = "center";
                row.style.gap = "10px"; // Add space between keys
                row.style.width = "100%"; // Ensure full width
            });
            
            // Fix individual keys
            const keys = keyboardControls.querySelectorAll('.key');
            keys.forEach(key => {
                key.style.width = "60px"; // Ensure keys are large enough
                key.style.height = "60px";
                key.style.fontSize = "1.2rem";
                key.style.flexShrink = "0"; // Prevent keys from shrinking
            });
            
            // Fix controls row layout (contains acceleration and steering controls)
            const controlsRow = keyboardControls.querySelector('.controls-row');
            if (controlsRow) {
                controlsRow.style.width = "100%";
                controlsRow.style.display = "flex";
                controlsRow.style.flexDirection = "column"; // Stack vertically to ensure everything fits
                controlsRow.style.gap = "20px";
                controlsRow.style.marginTop = "20px";
                controlsRow.style.boxSizing = "border-box";
                
                // Fix acceleration and steering controls
                const accelerationControls = controlsRow.querySelector('.acceleration-controls');
                const steeringControls = controlsRow.querySelector('.steering-controls');
                
                if (accelerationControls) {
                    accelerationControls.style.width = "100%";
                    accelerationControls.style.marginBottom = "15px";
                    
                    // Fix acceleration buttons
                    const accelerationButtons = accelerationControls.querySelectorAll('.acceleration-button');
                    accelerationButtons.forEach(button => {
                        button.style.width = "60px";
                        button.style.height = "60px";
                        button.style.fontSize = "0.9rem"; // Slightly smaller font for percentage text
                        button.style.padding = "0";
                        button.style.flexShrink = "0"; // Prevent buttons from shrinking
                    });
                }
                
                if (steeringControls) {
                    steeringControls.style.width = "100%";
                    
                    // Fix steering buttons
                    const steeringButtons = steeringControls.querySelectorAll('.steering-button');
                    steeringButtons.forEach(button => {
                        button.style.width = "60px";
                        button.style.height = "60px";
                        button.style.fontSize = "0.9rem"; // Slightly smaller font for degree text
                        button.style.padding = "0";
                        button.style.flexShrink = "0"; // Prevent buttons from shrinking
                    });
                }
            }
            
            // Fix controls description
            const controlsDescription = keyboardControls.querySelector('.controls-description');
            if (controlsDescription) {
                controlsDescription.style.width = "100%";
                controlsDescription.style.marginTop = "20px";
                
                // Fix control list items
                const controlItems = controlsDescription.querySelectorAll('li');
                controlItems.forEach(item => {
                    item.style.display = "flex";
                    item.style.alignItems = "center";
                    item.style.marginBottom = "10px";
                    item.style.fontSize = "0.9rem"; // Slightly smaller font for better fit
                    
                    // Fix key name elements
                    const keyName = item.querySelector('.key-name');
                    if (keyName) {
                        keyName.style.width = "30px";
                        keyName.style.height = "30px";
                        keyName.style.display = "flex";
                        keyName.style.justifyContent = "center";
                        keyName.style.alignItems = "center";
                        keyName.style.marginRight = "10px";
                        keyName.style.flexShrink = "0"; // Prevent from shrinking
                    }
                });
            }
        }
        
        // Fix racing wheel controls if present
        const wheelControls = controlsColumn.querySelector('.wheel-controls');
        if (wheelControls) {
            wheelControls.style.width = "100%";
            wheelControls.style.boxSizing = "border-box";
            
            // Fix steering wheel image
            const steeringWheel = wheelControls.querySelector('.steering-wheel');
            if (steeringWheel) {
                steeringWheel.style.width = "150px"; // Slightly smaller for better fit
                steeringWheel.style.height = "150px";
                steeringWheel.style.margin = "0 auto";
            }
            
            // Fix pedals
            const pedals = wheelControls.querySelector('.pedals');
            if (pedals) {
                pedals.style.display = "flex";
                pedals.style.justifyContent = "center";
                pedals.style.gap = "20px";
                pedals.style.marginTop = "15px";
            }
        }
    }
    
    // Position video column in the center with reduced side spacing
    if (videoColumn) {
        videoColumn.style.position = "fixed";
        videoColumn.style.top = "0";
        videoColumn.style.left = `${totalSideWidth}px`; // Position after left panel
        videoColumn.style.width = `calc(100vw - ${totalSideWidth * 2}px)`; // Width minus both side panels
        videoColumn.style.height = "100vh";
        videoColumn.style.display = "flex";
        videoColumn.style.justifyContent = "center";
        videoColumn.style.alignItems = "center";
        videoColumn.style.padding = "0";
        videoColumn.style.margin = "0";
        videoColumn.style.zIndex = "1000";
    }
    
    // Make video feeds container center content
    if (videoFeeds) {
        videoFeeds.style.position = "relative";
        videoFeeds.style.width = "100%";
        videoFeeds.style.height = "100%";
        videoFeeds.style.display = "flex";
        videoFeeds.style.justifyContent = "center";
        videoFeeds.style.alignItems = "center";
        videoFeeds.style.padding = "0";
        videoFeeds.style.margin = "0";
    }
    
    // Position primary feed to be centered
    if (primaryFeed) {
        primaryFeed.style.position = "relative";
        primaryFeed.style.width = "100%";
        primaryFeed.style.height = "100%";
        primaryFeed.style.display = "flex";
        primaryFeed.style.justifyContent = "center";
        primaryFeed.style.alignItems = "center";
        primaryFeed.style.margin = "0";
        primaryFeed.style.zIndex = "1000";
    }
    
    // Calculate aspect ratio and size based on height with slightly more space
    if (primaryFeedContainer && primaryFeedImg) {
        // Get the natural dimensions of the image
        const imgNaturalWidth = primaryFeedImg.naturalWidth || 640;
        const imgNaturalHeight = primaryFeedImg.naturalHeight || 480;
        const aspectRatio = imgNaturalWidth / imgNaturalHeight || 16/9;
        
        // Calculate the maximum size that fits in the available space
        // Reduced padding from 40px to 20px (10px on each side)
        const availableWidth = videoColumn.offsetWidth - 20;
        const availableHeight = window.innerHeight * 0.95;
        
        let containerWidth, containerHeight;
        
        // Determine if width or height is the limiting factor
        if (availableWidth / availableHeight < aspectRatio) {
            // Width is limiting factor
            containerWidth = availableWidth;
            containerHeight = containerWidth / aspectRatio;
        } else {
            // Height is limiting factor
            containerHeight = availableHeight;
            containerWidth = containerHeight * aspectRatio;
        }
        
        // Set container size to match the calculated dimensions
        primaryFeedContainer.style.position = "relative";
        primaryFeedContainer.style.width = `${containerWidth}px`;
        primaryFeedContainer.style.height = `${containerHeight}px`;
        primaryFeedContainer.style.maxHeight = "95vh";
        primaryFeedContainer.style.maxWidth = "100%";
        primaryFeedContainer.style.borderRadius = "8px";
        primaryFeedContainer.style.border = "2px solid var(--primary-color)";
        primaryFeedContainer.style.margin = "0 auto";
        primaryFeedContainer.style.overflow = "hidden";
        primaryFeedContainer.style.backgroundColor = originalBgColor;
        primaryFeedContainer.style.display = "flex";
        primaryFeedContainer.style.justifyContent = "center";
        primaryFeedContainer.style.alignItems = "center";
        
        // Set image to fill the container exactly
        primaryFeedImg.style.width = "100%";
        primaryFeedImg.style.height = "100%";
        primaryFeedImg.style.objectFit = "cover"; // Changed to cover to fill the container exactly
        primaryFeedImg.style.position = "relative";
        primaryFeedImg.style.display = "block";
        primaryFeedImg.style.margin = "0";
    }
    
    // Make sure the overlay stays aligned with the feed container
    if (primaryFeedOverlay) {
        primaryFeedOverlay.style.position = "absolute";
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
    
    // Update fullscreen button to show exit icon
    if (fullscreenBtn) {
        fullscreenBtn.textContent = "⤓"; // Change to exit fullscreen icon
        fullscreenBtn.title = "Exit Fullscreen";
        fullscreenBtn.style.zIndex = "1003"; // Ensure it's above other elements
        fullscreenBtn.style.fontSize = "1.2rem"; // Increased size for better visibility
    }
    
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
    mainContent.style.position = '';
    mainContent.style.top = '';
    mainContent.style.left = '';
    mainContent.style.width = '';
    mainContent.style.height = '';
    mainContent.style.padding = '';
    mainContent.style.margin = '';
    mainContent.style.backgroundColor = '';
    mainContent.style.display = '';
    mainContent.style.flexDirection = '';
    mainContent.style.justifyContent = '';
    mainContent.style.alignItems = '';
    mainContent.style.zIndex = '';
    
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
        videoColumn.style.zIndex = "";
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
        primaryFeed.style.zIndex = "";
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
        primaryFeedImg.style.display = "";
        primaryFeedImg.style.margin = "";
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
        secondaryFeed.style.right = "";
        secondaryFeed.style.transform = "";
        secondaryFeed.style.width = "";
        secondaryFeed.style.zIndex = "";
    }
    
    // Reset controls column styles
    if (controlsColumn) {
        controlsColumn.style.position = "";
        controlsColumn.style.top = "";
        controlsColumn.style.right = "";
        controlsColumn.style.transform = "";
        controlsColumn.style.width = "";
        controlsColumn.style.maxHeight = "";
        controlsColumn.style.overflowY = "";
        controlsColumn.style.margin = "";
        controlsColumn.style.padding = "";
        controlsColumn.style.opacity = "";
        controlsColumn.style.zIndex = "";
        controlsColumn.style.order = "";
        controlsColumn.style.boxSizing = "";
        
        // Reset all panels within the controls column
        const controlPanels = controlsColumn.querySelectorAll('.control-panel');
        controlPanels.forEach(panel => {
            panel.style.width = "";
            panel.style.boxSizing = "";
            panel.style.overflow = "";
            panel.style.margin = "";
            panel.style.padding = "";
        });
        
        // Reset keyboard controls
        const keyboardControls = controlsColumn.querySelector('.keyboard-controls');
        if (keyboardControls) {
            // Reset controls content
            const controlsContent = keyboardControls.querySelector('.controls-content');
            if (controlsContent) {
                controlsContent.style.width = "";
                controlsContent.style.padding = "";
                controlsContent.style.boxSizing = "";
            }
            
            // Reset keyboard wrapper
            const keyboardWrapper = keyboardControls.querySelector('.keyboard-controls-wrapper');
            if (keyboardWrapper) {
                keyboardWrapper.style.width = "";
                keyboardWrapper.style.boxSizing = "";
            }
            
            // Reset key rows
            const keyRows = keyboardControls.querySelectorAll('.key-row');
            keyRows.forEach(row => {
                row.style.marginBottom = "";
                row.style.display = "";
                row.style.justifyContent = "";
                row.style.gap = "";
                row.style.width = "";
            });
            
            // Reset individual keys
            const keys = keyboardControls.querySelectorAll('.key');
            keys.forEach(key => {
                key.style.width = "";
                key.style.height = "";
                key.style.fontSize = "";
                key.style.flexShrink = "";
            });
            
            // Reset controls row
            const controlsRow = keyboardControls.querySelector('.controls-row');
            if (controlsRow) {
                controlsRow.style.width = "";
                controlsRow.style.display = "";
                controlsRow.style.flexDirection = "";
                controlsRow.style.gap = "";
                controlsRow.style.marginTop = "";
                controlsRow.style.boxSizing = "";
                
                // Reset acceleration and steering controls
                const accelerationControls = controlsRow.querySelector('.acceleration-controls');
                const steeringControls = controlsRow.querySelector('.steering-controls');
                
                if (accelerationControls) {
                    accelerationControls.style.width = "";
                    accelerationControls.style.marginBottom = "";
                    
                    // Reset acceleration buttons
                    const accelerationButtons = accelerationControls.querySelectorAll('.acceleration-button');
                    accelerationButtons.forEach(button => {
                        button.style.width = "";
                        button.style.height = "";
                        button.style.fontSize = "";
                        button.style.padding = "";
                        button.style.flexShrink = "";
                    });
                }
                
                if (steeringControls) {
                    steeringControls.style.width = "";
                    
                    // Reset steering buttons
                    const steeringButtons = steeringControls.querySelectorAll('.steering-button');
                    steeringButtons.forEach(button => {
                        button.style.width = "";
                        button.style.height = "";
                        button.style.fontSize = "";
                        button.style.padding = "";
                        button.style.flexShrink = "";
                    });
                }
            }
            
            // Reset controls description
            const controlsDescription = keyboardControls.querySelector('.controls-description');
            if (controlsDescription) {
                controlsDescription.style.width = "";
                controlsDescription.style.marginTop = "";
                
                // Reset control list items
                const controlItems = controlsDescription.querySelectorAll('li');
                controlItems.forEach(item => {
                    item.style.display = "";
                    item.style.alignItems = "";
                    item.style.marginBottom = "";
                    item.style.fontSize = "";
                    
                    // Reset key name elements
                    const keyName = item.querySelector('.key-name');
                    if (keyName) {
                        keyName.style.width = "";
                        keyName.style.height = "";
                        keyName.style.display = "";
                        keyName.style.justifyContent = "";
                        keyName.style.alignItems = "";
                        keyName.style.marginRight = "";
                        keyName.style.flexShrink = "";
                    }
                });
            }
        }
        
        // Reset racing wheel controls
        const wheelControls = controlsColumn.querySelector('.wheel-controls');
        if (wheelControls) {
            wheelControls.style.width = "";
            wheelControls.style.boxSizing = "";
            
            // Reset steering wheel image
            const steeringWheel = wheelControls.querySelector('.steering-wheel');
            if (steeringWheel) {
                steeringWheel.style.width = "";
                steeringWheel.style.height = "";
                steeringWheel.style.margin = "";
            }
            
            // Reset pedals
            const pedals = wheelControls.querySelector('.pedals');
            if (pedals) {
                pedals.style.display = "";
                pedals.style.justifyContent = "";
                pedals.style.gap = "";
                pedals.style.marginTop = "";
            }
        }
    }
    
    // Reset status history column styles
    if (statusHistoryColumn) {
        statusHistoryColumn.style.position = "";
        statusHistoryColumn.style.left = "";
        statusHistoryColumn.style.top = "";
        statusHistoryColumn.style.transform = "";
        statusHistoryColumn.style.width = "";
        statusHistoryColumn.style.maxHeight = "";
        statusHistoryColumn.style.overflowY = "";
        statusHistoryColumn.style.margin = "";
        statusHistoryColumn.style.padding = "";
        statusHistoryColumn.style.opacity = "";
        statusHistoryColumn.style.zIndex = "";
        statusHistoryColumn.style.order = "";
        statusHistoryColumn.style.boxSizing = "";
        
        // Reset all panels within the status history column
        const statusPanels = statusHistoryColumn.querySelectorAll('.status-panel, .history-panel');
        statusPanels.forEach(panel => {
            panel.style.width = "";
            panel.style.boxSizing = "";
            panel.style.overflow = "";
            panel.style.margin = "";
            panel.style.padding = "";
        });
        
        // Reset status grid
        const statusGrid = statusHistoryColumn.querySelector('.status-grid');
        if (statusGrid) {
            statusGrid.style.gridTemplateColumns = "";
            statusGrid.style.width = "";
            statusGrid.style.boxSizing = "";
            statusGrid.style.gap = "";
            
            // Reset status items
            const statusItems = statusGrid.querySelectorAll('.status-item');
            statusItems.forEach(item => {
                item.style.width = "";
                item.style.boxSizing = "";
                item.style.minWidth = "";
                item.style.overflow = "";
                item.style.display = "";
                item.style.alignItems = "";
                item.style.padding = "";
                
                // Reset status values
                const statusValue = item.querySelector('.status-value');
                if (statusValue) {
                    statusValue.style.whiteSpace = "";
                    statusValue.style.overflow = "";
                    statusValue.style.textOverflow = "";
                    statusValue.style.fontSize = "";
                }
                
                // Reset status icons
                const statusIcon = item.querySelector('.status-icon');
                if (statusIcon) {
                    statusIcon.style.marginRight = "";
                    statusIcon.style.fontSize = "";
                }
            });
        }
        
        // Reset command log
        const commandLog = statusHistoryColumn.querySelector('.command-log');
        if (commandLog) {
            commandLog.style.width = "";
            commandLog.style.boxSizing = "";
            commandLog.style.height = "";
            commandLog.style.padding = "";
        }
    }
    
    // Reset fullscreen button to show fullscreen icon
    if (fullscreenBtn) {
        fullscreenBtn.textContent = "⛶"; // Change back to fullscreen icon
        fullscreenBtn.title = "Fullscreen";
        fullscreenBtn.style.zIndex = "";
        fullscreenBtn.style.fontSize = "";
    }
    
    // Show the header again
    const header = document.querySelector('.header');
    if (header) {
        header.style.display = '';
    }
    
    // Force a small delay and then trigger a resize event to help layout recalculation
    setTimeout(() => {
        window.dispatchEvent(new Event('resize'));
    }, 50);
    
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

// Add this function after the toggleFullscreen function
function setupResizeButtons() {
    console.log("Setting up resize buttons");
    
    // Get all resize buttons
    const resizeButtons = document.querySelectorAll('.resize-btn');
    
    resizeButtons.forEach(button => {
        // Remove existing event listeners by cloning and replacing
        const newButton = button.cloneNode(true);
        button.parentNode.replaceChild(newButton, button);
        
        // Add click event listener to the new button
        newButton.addEventListener('click', function(e) {
            e.preventDefault();
            e.stopPropagation();
            
            const action = this.getAttribute('data-action');
            const target = this.getAttribute('data-target');
            
            console.log(`Resize button clicked: ${action} for ${target}`);
            
            // Get the target feed container
            const feedContainer = document.querySelector(`.video-feed.${target} .feed-container`);
            if (!feedContainer) {
                console.error(`Feed container for ${target} not found`);
                return;
            }
            
            // Get current width
            const currentWidth = parseInt(getComputedStyle(feedContainer).width);
            let newWidth;
            
            // Calculate new width based on action
            if (action === 'increase') {
                newWidth = currentWidth * 1.1; // Increase by 10%
            } else if (action === 'decrease') {
                newWidth = currentWidth * 0.9; // Decrease by 10%
            }
            
            // Apply new width
            feedContainer.style.width = `${newWidth}px`;
            
            console.log(`Resized ${target} feed to ${newWidth}px`);
        });
    });
}

// Update the IIFE at the bottom of the file to include the resize buttons setup
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
        setupResizeButtons();
    }
    
    // Also set it up on DOMContentLoaded to be safe
    document.addEventListener('DOMContentLoaded', function() {
        setupFullscreenButton();
        setupEscapeKeyHandler();
        setupResizeButtons();
    });
    
    // And set it up on load as a final fallback
    window.addEventListener('load', function() {
        setupFullscreenButton();
        setupEscapeKeyHandler();
        setupResizeButtons();
    });
})();
