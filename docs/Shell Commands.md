| Command                | Description                                          | Example               |
| ---------------------- | ---------------------------------------------------- | --------------------- |
| ssh (ip)               | Used to remotely connect to a device with a known ip | ssh pi@192.168.1.1    |
| cd (path)              | Changes the current directory                        | cd /home/pi/Downloads |
| cd ../                 | Moves to the parent folder                           |                       |
| cd -                   | Return to the last cd (helpful for switching between two paths) |            
| ls                     | Lists all files in the current directory             |                       |
| nano                   | Text editor for various files                        | nano main.py          |
| clear                  | Clears all text from the shell                       |                       |
| rm                     | Removes a file                                       | rm delete_me.txt      |
| rm -r                  | Removes a folder with everything inside of it        | rm home/pi/delete_me  |
| sudo -s                | Elevates the terminal with administrator privilege   |                       |
| scp (file) (directory) | Copies files to another path or remote directory     |                       |
| nohup (command) (args) | Continues the command/executable even if ssh connection is lost|              |

| Shortcuts     | Description |
|---------------| ----------- |
| tab           | Autocompletes a path or filename |
| tab (x2)      | Shows all available autocompletions for a command |
| Ctrl + C      | Interrupt a currently running program |
| up/down arrow | Traverse through previously entered commands |

