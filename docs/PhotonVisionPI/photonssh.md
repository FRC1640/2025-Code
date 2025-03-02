# Photon Vision SSH
First, download [FileZilla](https://filezilla-project.org/) & [PuTTy](https://www.putty.org/). After that, SSH into the Orange Pi. Usually, the ip is
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
Now in filezilla, log in with the IP address of the device, ```root``` as the username, ```raspberry``` as the password, and ```22``` as the port.
Click through until you see the opt folder, click on it, click on photonvision, and then replace all of photonvision_config with the equivlent from the backups folder.
