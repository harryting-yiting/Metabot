# niryo_robot_database

## Requirements

* sqlite3  
  `sudo apt install sqlite3`
  
## Setup
### interactive way

```sql
sqlite3
.open /home/niryo/.config/niryo/ned.db
.read sql/create.sql
.read sql/populate.sql
```
press `ctrl+d` or type `.quit` to exit
### script way
```bash
./sql/init.sh
```