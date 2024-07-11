#!/bin/bash
# ~/bash_scripts/startup_handler.sh

FLAG_FILE="/tmp/first_login_flag"
TERMINAL_COUNT_FILE="/tmp/terminal_count"

# Function to initialize the terminal count file if it doesn't exist
initialize_terminal_count() {
    if [ ! -f "$TERMINAL_COUNT_FILE" ]; then
        echo 0 > "$TERMINAL_COUNT_FILE"
    fi
}

# Function to increment the terminal count
increment_terminal_count() {
    local count=$(cat "$TERMINAL_COUNT_FILE")
    echo $((count + 1)) > "$TERMINAL_COUNT_FILE"
}

# Function to check if it's the first terminal session
is_first_terminal() {
    local count=$(cat "$TERMINAL_COUNT_FILE")
    [ "$count" -eq 1 ]
}

# Function to reset the terminal count on shutdown
reset_terminal_count() {
    echo 0 > "$TERMINAL_COUNT_FILE"
}

# Initialize terminal count file
initialize_terminal_count
increment_terminal_count

# Run the script only on the first terminal login
if is_first_terminal; then
    if [ ! -f "$FLAG_FILE" ]; then
        bash ~/bash_scripts/all_run.sh
        touch "$FLAG_FILE"
    fi
fi

