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
    
    local service_count=${#services[@]}
    local setup_all_idx=$((service_count+1))
    local remove_one_idx=$((service_count+2))
    local remove_all_idx=$((service_count+3))
    local exit_idx=$((service_count+4))

    echo
    echo "  ${setup_all_idx}. Setup all services"
    echo "  ${remove_one_idx}. Remove a service"
    echo "  ${remove_all_idx}. Remove all services"
    echo "  ${exit_idx}. Exit"
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

# Function to remove a single service
remove_service() {
    local service_file="$1"
    local service_name=$(basename "$service_file")
    local systemd_path="/etc/systemd/system/$service_name"

    print_header "Removing $service_name..."

    if systemctl is-active --quiet "$service_name"; then
        print_status "Stopping $service_name..."
        if systemctl stop "$service_name"; then
            print_status "✓ Service stopped"
        else
            print_warning "✗ Failed to stop service"
            return 1
        fi
    else
        print_status "$service_name is not currently active"
    fi

    if systemctl list-unit-files "$service_name" >/dev/null 2>&1; then
        print_status "Disabling $service_name..."
        if systemctl disable "$service_name" >/dev/null 2>&1; then
            print_status "✓ Service disabled"
        else
            print_warning "✗ Failed to disable service"
            return 1
        fi
    else
        print_status "$service_name is not enabled"
    fi

    if [[ -f "$systemd_path" ]]; then
        print_status "Removing unit file at $systemd_path"
        if rm -f "$systemd_path"; then
            print_status "✓ Service file removed"
        else
            print_warning "✗ Failed to remove service file"
            return 1
        fi
    else
        print_status "No unit file found at $systemd_path"
    fi

    print_status "Reloading systemd daemon..."
    if systemctl daemon-reload; then
        print_status "✓ Systemd daemon reloaded"
    else
        print_warning "✗ Failed to reload systemd daemon"
        return 1
    fi

    print_status "Resetting failed state (if any)..."
    if systemctl reset-failed "$service_name" >/dev/null 2>&1; then
        print_status "✓ Failed state reset"
    else
        print_warning "✗ Failed to reset state"
    fi

    echo
    return 0
}

# Function to remove selected services
remove_services() {
    local selected_services=("$@")
    local success_count=0
    local total_count=${#selected_services[@]}

    for service_file in "${selected_services[@]}"; do
        if remove_service "$service_file"; then
            ((success_count++))
        fi
    done

    print_header "=== Removal Summary ==="
    print_status "Successfully removed $success_count out of $total_count services"

    if [[ $success_count -lt $total_count ]]; then
        print_warning "Some services could not be removed. Check the output above for details."
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

        local service_count=${#services[@]}
        local setup_all_option=$((service_count+1))
        local remove_one_option=$((service_count+2))
        local remove_all_option=$((service_count+3))
        local exit_option=$((service_count+4))

        case $choice in
            $exit_option)
                print_status "Exiting..."
                exit 0
                ;;
            $setup_all_option)
                print_status "Setting up all services..."
                setup_services "${services[@]}"
                break
                ;;
            *)
                if [[ $choice -ge 1 && $choice -le $service_count ]]; then
                    local selected_service="${services[$((choice-1))]}"
                    setup_services "$selected_service"
                    break
                elif [[ $choice -eq $remove_one_option ]]; then
                    while true; do
                        read -p "Enter the number of the service to remove (or 'b' to go back): " remove_choice
                        if [[ "$remove_choice" =~ ^[Bb]$ ]]; then
                            break
                        elif [[ $remove_choice -ge 1 && $remove_choice -le $service_count ]]; then
                            local remove_service_file="${services[$((remove_choice-1))]}"
                            remove_services "$remove_service_file"
                            exit 0
                        else
                            print_error "Invalid option. Please try again."
                        fi
                    done
                elif [[ $choice -eq $remove_all_option ]]; then
                    print_status "Removing all services..."
                    remove_services "${services[@]}"
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
