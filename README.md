# ups3

This repository is a fork of a fork from the original 
[Raspberry Pi UPS HAT V3](https://github.com/geekworm-com/ups3).
Owners email contact: info@geekworm.com

## Fork by as207564

[As207564](https://github.com/as207564/ups3) forked the repository with the focus of making it compatible with python3.

Systemctl service was originally configured to use the default 'pi' user.
This did however not work, as the user 'pi' did not have the sufficient rights to read from i2c (i guess; i'm no expert...). User was changed to 'root'.

Script's was also not using the python3 print() syntax, so this was corrected as well.

## Fork by cdgeorge

My [Cdgeorge](https://github.com/cdgeorge/ups3) fork has the main focus to make the UPS work as expected, by addressing some of the issues in the origianl software.

1. Bugfix: Repluging the rpi powersupply does no longer result in a slow loose of power, due to a current limit of 500 mA at 5V.
2. Limit the amount of logging, only log on change.
3. Added an estimate of time left before shutdowning when discarging.
4. Added signal handling.
5. Corrected execption handling.
6. Added support for dumpling all registers of BQ25895, without restarting the service.
7. Maded the status.py python3 compatable.
8. Formatted and aligned smartups.py and status.py
9. Added systemctl daemon-reload to install script

### Missing
Fails a cupple of times on start up

# Test
This project has been tested on a Raspberry Pi 4 (Buster) system.

# Setup (Raspberry Pi)

```shell
git clone https://github.com/geekworm-com/ups3.git
cd ups3
chmod +x *.sh *.py
sudo ./install.sh
```

               ┌────────────────────┤ UPS V3 Setting ├────────────────────┐
               │ Select the appropriate options:                          │
               │                                                          │
               │                 1 UPS GPIO [ 18 ]                        │
               │                 2 LED Brightness [ 10% ]                 │
               │                 3 Poweoff power [ <5% ]                  │
               │                 4 Auto run script [ enabled ]            │
               │                 5 Safe shutdown [ enabled ]              │
               │                 6 Apply Settings                         │
               │                 7 Remove                                 │
               │                 8 Exit                                   │
               │                                                          │
               │                                                          │
               │                                                          │
               │                                                          │
               │                          <Ok>                            │
               │                                                          │
               └──────────────────────────────────────────────────────────┘
               

Description of each option：

After you select an option, you must use option 6 to make it effective！！

1 UPS GPIO [ 18 ] :

Ups3 uses GPIO18 to manage LED lights by default, When you need to use GPIO 18 yourself, through this option, you can modify the GPIO PIN

2 LED Brightness [ 10% ]:

You can change the brightness of LED via this option, you need to reboot raspberry pi to take effect.

3 Poweoff power [ <5% ]：

This is a useful function, you can set the percentage of power to tell UPS3 to shut down automatically

4 Autorun [ enabled ] :

If you want this service to run automatically every time the Raspberry Pi restarts, please remember to enable it.

5 Safe shutdown [ enabled ] :

Please refer to here: https://wiki.geekworm.com/UPS3_power_off_guide

6 Apply Settings:

After you select an option, you must use this option to make it effective;

7 Remove:

Remove or uninstall this script / service;

8 Exit:

Exit this menu;

View Status:

`sudo python status.py` or `sudo python status.py -t`

View logs:

`cat /var/log/smartups.log`

