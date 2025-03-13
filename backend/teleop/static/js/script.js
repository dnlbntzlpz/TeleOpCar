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
    
    // Hide resize buttons in fullscreen mode
    const resizeButtons = primaryFeedOverlay ? primaryFeedOverlay.querySelectorAll('.resize-btn') : null;
    if (resizeButtons) {
        resizeButtons.forEach(button => {
            button.style.display = 'none';
        });
    }
    
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
    mainContent.style.justifyContent = 'center'; // Center content
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
    const sideColumnWidth = 400;
    const sideColumnMargin = 10;
    
    // Make video column fill the entire viewport
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
    
    // Make video feeds container fill the entire space
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
    
    // Make primary feed fill the entire viewport
    if (primaryFeed) {
        primaryFeed.style.position = "absolute";
        primaryFeed.style.top = "0";
        primaryFeed.style.left = "0";
        primaryFeed.style.width = "100%";
        primaryFeed.style.height = "100%";
        primaryFeed.style.display = "flex";
        primaryFeed.style.justifyContent = "center";
        primaryFeed.style.alignItems = "center";
        primaryFeed.style.margin = "0";
        primaryFeed.style.zIndex = "1000";
    }
    
    // Make primary feed container fill the entire viewport
    if (primaryFeedContainer && primaryFeedImg) {
        primaryFeedContainer.style.position = "absolute";
        primaryFeedContainer.style.top = "0";
        primaryFeedContainer.style.left = "0";
        primaryFeedContainer.style.width = "100%";
        primaryFeedContainer.style.height = "100%";
        primaryFeedContainer.style.maxHeight = "none";
        primaryFeedContainer.style.maxWidth = "none";
        primaryFeedContainer.style.borderRadius = "0";
        primaryFeedContainer.style.border = "none";
        primaryFeedContainer.style.margin = "0";
        primaryFeedContainer.style.overflow = "hidden";
        primaryFeedContainer.style.backgroundColor = originalBgColor;
        primaryFeedContainer.style.display = "flex";
        primaryFeedContainer.style.justifyContent = "center";
        primaryFeedContainer.style.alignItems = "center";
        
        // Set image to fill the container exactly
        primaryFeedImg.style.width = "100%";
        primaryFeedImg.style.height = "100%";
        primaryFeedImg.style.objectFit = "cover"; // Fill the entire container
        primaryFeedImg.style.position = "absolute";
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
        feedLabel.style.fontSize = "1.2rem";
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
    
    // Position secondary feed at the top center with increased size
    if (secondaryFeed) {
        secondaryFeed.style.position = "fixed";
        secondaryFeed.style.top = "20px";
        secondaryFeed.style.left = "50%";
        secondaryFeed.style.transform = "translateX(-50%)";
        secondaryFeed.style.width = "400px"; // Increased from 300px to 400px
        secondaryFeed.style.zIndex = "1002";
        
        // Make sure the feed container and image are properly sized
        const secondaryFeedContainer = secondaryFeed.querySelector('.feed-container');
        if (secondaryFeedContainer) {
            secondaryFeedContainer.style.width = "100%";
            secondaryFeedContainer.style.height = "auto";
            
            // Ensure the image fills the container
            const secondaryFeedImg = secondaryFeedContainer.querySelector('img');
            if (secondaryFeedImg) {
                secondaryFeedImg.style.width = "100%";
                secondaryFeedImg.style.height = "100%";
                secondaryFeedImg.style.objectFit = "cover";
            }
        }
    }
    
    // Position status history column on the left as an overlay
    if (statusHistoryColumn) {
        statusHistoryColumn.style.position = "fixed";
        statusHistoryColumn.style.left = `${sideColumnMargin}px`;
        statusHistoryColumn.style.top = "50%";
        statusHistoryColumn.style.transform = "translateY(-50%)";
        statusHistoryColumn.style.width = `${sideColumnWidth}px`;
        statusHistoryColumn.style.maxHeight = "95vh";
        statusHistoryColumn.style.overflowY = "auto";
        statusHistoryColumn.style.margin = "0";
        statusHistoryColumn.style.opacity = "0.75"; // Changed to 0.25 (25% opacity, 75% transparent)
        statusHistoryColumn.style.zIndex = "1002";
        statusHistoryColumn.style.padding = "10px";
        statusHistoryColumn.style.boxSizing = "border-box";
        statusHistoryColumn.style.backdropFilter = "blur(5px)"; // Add blur effect behind the panel
        
        // Add hover effect to make it fully opaque when hovered
        statusHistoryColumn.addEventListener('mouseenter', function() {
            this.style.opacity = "1";
        });
        statusHistoryColumn.addEventListener('mouseleave', function() {
            this.style.opacity = "0.75"; // Changed to 0.25
        });
        
        // Specifically target the vehicle status panel to make it taller
        const vehicleStatusPanel = statusHistoryColumn.querySelector('.status-panel');
        if (vehicleStatusPanel) {
            vehicleStatusPanel.style.minHeight = "520px";
            vehicleStatusPanel.style.height = "auto";
        }
        
        // Ensure status grid items have proper width and don't get cut off
        const statusGrid = statusHistoryColumn.querySelector('.status-grid');
        if (statusGrid) {
            statusGrid.style.gridTemplateColumns = "1fr 1fr";
            statusGrid.style.width = "100%";
            statusGrid.style.boxSizing = "border-box";
            statusGrid.style.gap = "15px";
            statusGrid.style.padding = "5px";
            
            // Make sure each status item has enough space
            const statusItems = statusGrid.querySelectorAll('.status-item');
            statusItems.forEach(item => {
                item.style.width = "100%";
                item.style.boxSizing = "border-box";
                item.style.minWidth = "0";
                item.style.overflow = "hidden";
                item.style.display = "flex";
                item.style.alignItems = "center";
                item.style.padding = "12px";
                item.style.marginBottom = "10px";
                
                // Ensure status values don't get cut off
                const statusValue = item.querySelector('.status-value');
                if (statusValue) {
                    statusValue.style.whiteSpace = "nowrap";
                    statusValue.style.overflow = "hidden";
                    statusValue.style.textOverflow = "ellipsis";
                    statusValue.style.fontSize = "1.1rem";
                }
                
                // Ensure status icons have enough space
                const statusIcon = item.querySelector('.status-icon');
                if (statusIcon) {
                    statusIcon.style.marginRight = "10px";
                    statusIcon.style.fontSize = "1.5rem";
                }
            });
        }
    }
    
    // Position controls column on the right as an overlay
    if (controlsColumn) {
        controlsColumn.style.position = "fixed";
        controlsColumn.style.right = `${sideColumnMargin}px`;
        controlsColumn.style.top = "50%";
        controlsColumn.style.transform = "translateY(-50%)";
        controlsColumn.style.width = `${sideColumnWidth}px`;
        controlsColumn.style.maxHeight = "95vh";
        controlsColumn.style.overflowY = "auto";
        controlsColumn.style.margin = "0";
        controlsColumn.style.opacity = "0.75";
        controlsColumn.style.zIndex = "1002";
        controlsColumn.style.padding = "10px";
        controlsColumn.style.boxSizing = "border-box";
        controlsColumn.style.backdropFilter = "blur(5px)";
        
        // Add hover effect to make it fully opaque when hovered
        controlsColumn.addEventListener('mouseenter', function() {
            this.style.opacity = "1";
        });
        controlsColumn.addEventListener('mouseleave', function() {
            this.style.opacity = "0.75";
        });
        
        // Fix for manual controls layout - ensure steering buttons are visible
        const keyboardControls = controlsColumn.querySelector('.keyboard-controls');
        if (keyboardControls) {
            // Ensure the controls content has proper width
            const controlsContent = keyboardControls.querySelector('.controls-content');
            if (controlsContent) {
                controlsContent.style.width = "100%";
                controlsContent.style.padding = "10px";
                controlsContent.style.boxSizing = "border-box";
            }
            
            // Fix the controls row layout
            const controlsRow = keyboardControls.querySelector('.controls-row');
            if (controlsRow) {
                controlsRow.style.width = "100%";
                controlsRow.style.display = "flex";
                controlsRow.style.flexDirection = "column"; // Stack controls vertically
                controlsRow.style.gap = "15px";
                controlsRow.style.marginTop = "15px";
                
                // Fix acceleration controls - ensure title is above buttons
                const accelerationControls = controlsRow.querySelector('.acceleration-controls');
                if (accelerationControls) {
                    accelerationControls.style.width = "100%";
                    accelerationControls.style.marginBottom = "20px";
                    accelerationControls.style.display = "flex";
                    accelerationControls.style.flexDirection = "column"; // Stack title above buttons
                    accelerationControls.style.alignItems = "center";
                    
                    // Style the title if it exists
                    const accelerationTitle = accelerationControls.querySelector('.acceleration-title');
                    if (accelerationTitle) {
                        accelerationTitle.style.marginBottom = "10px";
                        accelerationTitle.style.textAlign = "center";
                        accelerationTitle.style.width = "100%";
                        accelerationTitle.style.color = "var(--text-secondary, #a0b0c0)";
                    }
                    
                    // Style the button container
                    const accelerationButtonsContainer = accelerationControls.querySelector('.acceleration-buttons');
                    if (accelerationButtonsContainer) {
                        accelerationButtonsContainer.style.display = "flex";
                        accelerationButtonsContainer.style.flexDirection = "row";
                        accelerationButtonsContainer.style.justifyContent = "space-between";
                        accelerationButtonsContainer.style.width = "100%";
                    } else {
                        // If there's no container, style the buttons directly
                    const accelerationButtons = accelerationControls.querySelectorAll('.acceleration-button');
                        // Create a container for the buttons
                        if (accelerationButtons.length > 0 && !accelerationButtonsContainer) {
                            const buttonsDiv = document.createElement('div');
                            buttonsDiv.className = 'acceleration-buttons';
                            buttonsDiv.style.display = "flex";
                            buttonsDiv.style.flexDirection = "row";
                            buttonsDiv.style.justifyContent = "space-between";
                            buttonsDiv.style.width = "100%";
                            
                            // Move buttons into the container
                    accelerationButtons.forEach(button => {
                        button.style.flexShrink = "0";
                                accelerationControls.removeChild(button);
                                buttonsDiv.appendChild(button);
                            });
                            
                            // Add the container after the title
                            if (accelerationTitle) {
                                accelerationTitle.insertAdjacentElement('afterend', buttonsDiv);
                            } else {
                                accelerationControls.appendChild(buttonsDiv);
                            }
                        }
                    }
                }
                
                // Fix steering controls - ensure title is above buttons
                const steeringControls = controlsRow.querySelector('.steering-controls');
                if (steeringControls) {
                    steeringControls.style.width = "100%";
                    steeringControls.style.display = "flex";
                    steeringControls.style.flexDirection = "column"; // Stack title above buttons
                    steeringControls.style.alignItems = "center";
                    
                    // Style the title if it exists
                    const steeringTitle = steeringControls.querySelector('.steering-title');
                    if (steeringTitle) {
                        steeringTitle.style.marginBottom = "10px";
                        steeringTitle.style.textAlign = "center";
                        steeringTitle.style.width = "100%";
                        steeringTitle.style.color = "var(--text-secondary, #a0b0c0)";
                    }
                    
                    // Style the button container
                    const steeringButtonsContainer = steeringControls.querySelector('.steering-buttons');
                    if (steeringButtonsContainer) {
                        steeringButtonsContainer.style.display = "flex";
                        steeringButtonsContainer.style.flexDirection = "row";
                        steeringButtonsContainer.style.justifyContent = "space-between";
                        steeringButtonsContainer.style.width = "100%";
                    } else {
                        // If there's no container, style the buttons directly
                        const steeringButtons = steeringControls.querySelectorAll('.steering-button');
                        // Create a container for the buttons
                        if (steeringButtons.length > 0 && !steeringButtonsContainer) {
                            const buttonsDiv = document.createElement('div');
                            buttonsDiv.className = 'steering-buttons';
                            buttonsDiv.style.display = "flex";
                            buttonsDiv.style.flexDirection = "row";
                            buttonsDiv.style.justifyContent = "space-between";
                            buttonsDiv.style.width = "100%";
                            
                            // Move buttons into the container
                            steeringButtons.forEach(button => {
                                button.style.flexShrink = "0";
                                steeringControls.removeChild(button);
                                buttonsDiv.appendChild(button);
                            });
                            
                            // Add the container after the title
                            if (steeringTitle) {
                                steeringTitle.insertAdjacentElement('afterend', buttonsDiv);
                            } else {
                                steeringControls.appendChild(buttonsDiv);
                            }
                        }
                    }
                }
            }
        }
    }
    
    // Update fullscreen button to show exit icon
    if (fullscreenBtn) {
        fullscreenBtn.textContent = "⤓";
        fullscreenBtn.title = "Exit Fullscreen";
        fullscreenBtn.style.zIndex = "1003";
        fullscreenBtn.style.fontSize = "1.2rem";
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
    
    // Show resize buttons when exiting fullscreen mode
    const resizeButtons = primaryFeedOverlay ? primaryFeedOverlay.querySelectorAll('.resize-btn') : null;
    if (resizeButtons) {
        resizeButtons.forEach(button => {
            button.style.display = '';
        });
    }
    
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
        fullscreenBtn.textContent = "⛶";
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

// Add this function to handle telemetry updates
function setupTelemetryUpdates() {
    console.log("Setting up telemetry updates");
    
    // Function to fetch and update telemetry data
function updateTelemetry() {
        fetch('/get_telemetry/')
        .then(response => response.json())
        .then(data => {
            console.log("Received telemetry data:", data);
            
            // Update status values
            if (data.temperature) {
                document.getElementById('status-temperature').textContent = `${data.temperature}°C`;
            }
            if (data.battery) {
                document.getElementById('status-battery').textContent = `${data.battery}%`;
            }
            if (data.latency) {
                document.getElementById('status-latency').textContent = `${data.latency}ms`;
            }
            if (data.fps) {
                document.getElementById('status-fps').textContent = `${data.fps}`;
            }
            if (data.connection) {
                document.getElementById('status-connection').textContent = data.connection;
            }
            if (data.signal) {
                document.getElementById('status-signal').textContent = `${data.signal}%`;
            }
            if (data.gamepad) {
                document.getElementById('status-controller').textContent = data.gamepad;
            }
            if (data.cpu_load) {
                document.getElementById('status-cpu').textContent = `${data.cpu_load}%`;
            }
        })
        .catch(error => {
            console.error("Error fetching telemetry:", error);
        });
    }
    
    // Update telemetry immediately
updateTelemetry();
    
    // Set up interval to update telemetry every 15 seconds
    const telemetryInterval = setInterval(updateTelemetry, 15000);
    
    // Store the interval ID on the window object so we can clear it if needed
    window.telemetryInterval = telemetryInterval;
}

// Update the IIFE at the bottom of the file to include the resize buttons setup and telemetry updates
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
        setupTelemetryUpdates();
    }
    
    // Also set it up on DOMContentLoaded to be safe
    document.addEventListener('DOMContentLoaded', function() {
        setupFullscreenButton();
        setupEscapeKeyHandler();
        setupResizeButtons();
        setupTelemetryUpdates();
    });
    
    // And set it up on load as a final fallback
    window.addEventListener('load', function() {
        setupFullscreenButton();
        setupEscapeKeyHandler();
        setupResizeButtons();
        setupTelemetryUpdates();
    });
})();
