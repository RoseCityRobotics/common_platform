#!/bin/bash

# Setup script for nginx to serve Common Platform documentation
# Also handles documentation updates
# Run this script with sudo for initial setup, without sudo for updates

# Function to update documentation
update_docs() {
    echo "Updating Common Platform documentation..."
    
    # Build the documentation
    echo "Building documentation..."
    cd /home/rcr/repos/common_platform/docs
    make html
    
    if [ $? -eq 0 ]; then
        echo "✅ Documentation built successfully!"
        
        # Copy to web directory
        echo "Copying to web directory..."
        sudo cp -r build/html/* /var/www/html/
        
        echo "✅ Documentation updated on website!"
        echo "📚 Available at: http://localhost"
    else
        echo "❌ Documentation build failed!"
        exit 1
    fi
}

# Check if this is an update request
if [ "$1" = "update" ]; then
    update_docs
    exit 0
fi

# Initial setup
echo "Setting up nginx for Common Platform documentation..."

# Check if nginx is installed
if ! command -v nginx &> /dev/null; then
    echo "Installing nginx..."
    sudo apt update
    sudo apt install -y nginx
fi

# Build documentation first
echo "Building initial documentation..."
cd /home/rcr/repos/common_platform/docs
make html

if [ $? -ne 0 ]; then
    echo "❌ Failed to build documentation!"
    exit 1
fi

# Copy documentation to web directory
echo "Copying documentation to web directory..."
sudo cp -r build/html/* /var/www/html/

# Create nginx configuration
echo "Creating nginx configuration..."
sudo cp nginx-common-platform.conf /etc/nginx/sites-available/common-platform

# Enable the site
echo "Enabling the site..."
sudo ln -sf /etc/nginx/sites-available/common-platform /etc/nginx/sites-enabled/

# Remove default nginx site if it exists
if [ -f /etc/nginx/sites-enabled/default ]; then
    echo "Removing default nginx site..."
    sudo rm /etc/nginx/sites-enabled/default
fi

# Test nginx configuration
echo "Testing nginx configuration..."
sudo nginx -t

if [ $? -eq 0 ]; then
    echo "Nginx configuration is valid. Restarting nginx..."
    sudo systemctl restart nginx
    sudo systemctl enable nginx
    
    echo "✅ Nginx setup complete!"
    echo "📚 Documentation is available at:"
    echo "   http://localhost"
    echo "   http://$(hostname -I | awk '{print $1}')"
    echo ""
    echo "To update documentation after changes:"
    echo "   ./setup-nginx.sh update"
else
    echo "❌ Nginx configuration test failed. Please check the configuration."
    exit 1
fi
