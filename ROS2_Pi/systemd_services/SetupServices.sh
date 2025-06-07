#!/bin/bash

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}$1${NC}"
}

# Function to check if running as root
check_root() {
    if [[ $EUID -ne 0 ]]; then
        print_error "This script must be run as root (use sudo)"
        exit 1
    fi
}

# Function to scan for .service files
scan_services() {
    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    mapfile -t services < <(find "$script_dir" -name "*.service" -type f | sort)
    
    if [[ ${#services[@]} -eq 0 ]]; then
        print_error "No .service files found in $script_dir"
        exit 1
    fi
    
    print_status "Found ${#services[@]} service file(s)"
}

# Function to display menu
show_menu() {
    print_header "=== Systemd Service Setup Menu ==="
    echo
    
    for i in "${!services[@]}"; do
        local service_name=$(basename "${services[$i]}")
        echo "  $((i+1)). $service_name"
    done
    
    echo "  $((${#services[@]}+1)). All services"
    echo "  $((${#services[@]}+2)). Exit"
    echo
}

# Function to setup a single service
setup_service() {
    local service_file="$1"
    local service_name=$(basename "$service_file")
    
    print_header "Setting up $service_name..."
    
    # Step 1: Copy to systemd directory
    print_status "Copying $service_name to /etc/systemd/system/"
    if cp "$service_file" /etc/systemd/system/; then
        print_status "✓ Service file copied successfully"
    else
        print_error "✗ Failed to copy service file"
        return 1
    fi
    
    # Step 2: Reload systemd daemon
    print_status "Reloading systemd daemon..."
    if systemctl daemon-reload; then
        print_status "✓ Systemd daemon reloaded"
    else
        print_error "✗ Failed to reload systemd daemon"
        return 1
    fi
    
    # Step 3: Enable the service
    print_status "Enabling $service_name..."
    if systemctl enable "$service_name"; then
        print_status "✓ Service enabled"
    else
        print_error "✗ Failed to enable service"
        return 1
    fi
    
    # Step 4: Start the service
    print_status "Starting $service_name..."
    if systemctl start "$service_name"; then
        print_status "✓ Service started"
    else
        print_error "✗ Failed to start service"
        return 1
    fi
    
    # Step 5: Verify service status
    print_status "Verifying service status..."
    if systemctl is-active --quiet "$service_name"; then
        print_status "✓ Service is running correctly"
        systemctl status "$service_name" --no-pager -l
    else
        print_warning "⚠ Service may not be running correctly"
        systemctl status "$service_name" --no-pager -l
        return 1
    fi
    
    echo
    return 0
}

# Function to setup selected services
setup_services() {
    local selected_services=("$@")
    local success_count=0
    local total_count=${#selected_services[@]}
    
    for service_file in "${selected_services[@]}"; do
        if setup_service "$service_file"; then
            ((success_count++))
        fi
    done
    
    print_header "=== Setup Summary ==="
    print_status "Successfully set up $success_count out of $total_count services"
    
    if [[ $success_count -lt $total_count ]]; then
        print_warning "Some services failed to set up properly. Check the output above for details."
        return 1
    fi
}

# Main function
main() {
    check_root
    scan_services
    
    while true; do
        show_menu
        read -p "Select an option: " choice
        
        case $choice in
            $((${#services[@]}+2)))
                print_status "Exiting..."
                exit 0
                ;;
            $((${#services[@]}+1)))
                print_status "Setting up all services..."
                setup_services "${services[@]}"
                break
                ;;
            *)
                if [[ $choice -ge 1 && $choice -le ${#services[@]} ]]; then
                    local selected_service="${services[$((choice-1))]}"
                    setup_services "$selected_service"
                    break
                else
                    print_error "Invalid option. Please try again."
                fi
                ;;
        esac
    done
}

# Run main function
main "$@"
