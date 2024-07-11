#!/bin/bash

cd ~/ardupilot

# Create the expect script
expect_script=$(cat <<EOF
#!/usr/bin/expect -f

# Run mavproxy.py
spawn mavproxy.py

# Define the command(s) you want to send
set commands {
    "param set DDS_ENABLE 1"
    "reboot"
    "reboot"
}

# Function to send commands with delay
proc send_commands {} {
    global commands
    foreach cmd \$commands {
        send -- "\$cmd\r"
        sleep 8  ;# Add a delay of 8 seconds between commands
        expect "STABILIZE>"
    }
}

# Wait for the program to load after reboot
sleep 8  ;# Initial delay for the program to load

# Press enter to continue to the STABILIZE> prompt
send -- "\r"
expect "STABILIZE>"

# Send the commands with delay
send_commands

# Keep the session open
interact
EOF
)

# Save the expect script to a temporary file
expect_script_file=$(mktemp)
echo "$expect_script" > "$expect_script_file"
chmod +x "$expect_script_file"

# Run the expect script
"$expect_script_file"

# Clean up
rm "$expect_script_file"

