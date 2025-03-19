# Photon Vision SSH
First, download [FileZilla](https://filezilla-project.org/) & [PuTTy](https://www.putty.org/). After that, SSH into the Orange Pi (or other SBC). Usually, the ip is
```
10.16.40.63
```
(but this should be confirmed) and the port is ```22```. The username and password should be the default
```
Username: pi
Password: raspberry
```
Once in, type
```
sudo passwd root
```
This lets you change the password of the root user. Type in raspberry when prompted for a new password. Nothing will show up when typing in these feilds.
Next type
```
sudo passwd -u root
```
to enable the root account.
Finaly type
```
sudo nano /etc/ssh/sshd_config
```
Find the line that says
```
#PermitRootLogin prohibit-password
```
Delete the hashtag and change it to
```
PermitRootLogin yes
```
Press ctrl O to write out, and enter to confirm. Then press ctrl X to exit.
Finaly type:
```
sudo systemctl restart ssh
```
Now in filezilla, log in with the IP address of the device, ```root``` as the username, ```raspberry``` as the password, and ```22``` as the port. Then press ```Quick Connect```
Click navigate to /opt/photonvision on the left side (in the SBC's file system). You should see a folder entitled ```photonvision_config```.
Navigate to the robot code project, and open the backups folder. Here, you should also see a folder entitled ```photonvision_config```.
Finaly, drag the folder ```photonvision_config``` folder from the robot code on to the SBC, to replace its ```photonvision_config``` folder.
Confirm the files transfered by opening the ```photonvision_config``` folder on the SBC and confirming the sqlite file is more than one megabyte.
