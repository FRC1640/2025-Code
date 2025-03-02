# Photon Vision
**Install for SBC**

Reasources needed:
- board to run photonvision
- micro SD card
- Balena Etcher: (install at https://etcher.balena.io/)

Head to: https://photonvision.org/downloads.html and navigate to github page (as of writing, click on the Latest Release link).

Download the image for your SBC.

Insert micro SD card into your computer, and flash using Balena Etcher, run as administrator.
- Select the photon vision image as the file.
- Select the SD card as the drive.
- Click flash and wait for the sweet "successful flash" message.

When complete insert the card into the SBC. Congrats. Your Done! You can acsess the web ui at photonvision.local.



**Limelight Install**

Step 1: Put on some music. https://youtu.be/ZiRuj2_czzw

Step 2: Install Photon Vision

Head to: https://photonvision.org/downloads.html and navigate to github page (as of writing, click on the Latest Release link).

Download the image for your Limelight from the latest releases.

Download USB drivers from https://docs.limelightvision.io/docs/resources/downloads

Hold the config button (the only button) while you plug it in to a usb c cable connected to your computer.
The limelight will power on.

Run "rpiboot-CM4-CM5 - Mass Storage Gadget" from the windows start menu.

Run Balena Etcher as Administrator.
- Select the photon vision image as the file.
- Select the Internal Storage of the limelight as the drive.
- Click flash and wait for the sweet "successful flash" message.

When complete insert the card into the SBC. Congrats. Your Done! You can acsess the web ui at photonvision.local.
