# Installing Docker on Windows

This guide provides step-by-step instructions to install Docker on a Windows system.

## Step 1: Check System Requirements

Ensure your system meets the following requirements:

- Windows 10 64-bit: Pro, Enterprise, or Education (Build 15063 or later) or Windows 11
- Enable the WSL 2 feature on Windows. Docker Desktop requires the Windows Subsystem for Linux 2 (WSL 2) to run.

## Step 2: Install Git

1. **Check if Git is installed:**
   Open PowerShell and type:

   ```powershell
   git --version
   ```

   If Git is installed, you will see the version number. If not, proceed to the next step.

2. **Download Git for Windows:**
   Go to the [Git for Windows](https://gitforwindows.org/) page and click "Download".

3. **Run the Installer:**
   Run the downloaded installer and follow the instructions in the setup wizard. You can use the default settings for most options.

4. **Verify the Installation:**
   Open PowerShell and run:
   ```powershell
   git --version
   ```
   You should see the Git version number if the installation was successful.

## Step 3: Fork the Repository on GitHub

1. **Go to the GitHub repository:**
   Open your web browser and navigate to the repository URL.

2. **Fork the repository:**
   Click the "Fork" button on the top-right corner of the page. This will create a copy of the repository in your GitHub account.

3. **Clone the forked repository:**
   Open Terminal and run the following command, replacing `<your-username>` with your GitHub username:

   ```sh
   git clone git@github.com:<your-username>/ROS_Software_Assignement.git
   ```

   Navigate to the cloned repository:

   ```sh
   cd ROS_Software_Assignement
   ```

## Step 4: Enable WSL 2 Feature

1. **Enable WSL 1:**

   - Open PowerShell as an Administrator and run the following command:
     ```powershell
     dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
     ```

2. **Enable Virtual Machine Platform:**

   - Run the following command in PowerShell:
     ```powershell
     dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
     ```

3. **Set WSL 2 as the Default Version:**

   - Run the following command in PowerShell:
     ```powershell
     wsl --set-default-version 2
     ```

4. **Install a Linux Distribution:**
   - Go to the Microsoft Store, search for "Linux" or "Ubuntu", and install your preferred Linux distribution.

## Step 5: Download Docker Desktop for Windows

> [!NOTE]  
> If you have troubles with the installation, check the Docker [Installation](https://docs.docker.com/desktop/install/windows-install/) page for Windows.

1. **Download Docker Desktop:**

   - Go to the [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop) page and click "Download for Windows".

2. **Run the Installer:**
   - Run the downloaded installer. Follow the installation instructions, and make sure to enable the WSL 2 feature in the configuration step.

## Step 4: Install Docker Desktop

1. **Run the Installer:**

   - Double-click `Docker Desktop Installer.exe` to run the installer. If you haven’t already downloaded the installer (Docker Desktop Installer.exe), you can get it from [Docker Hub](https://hub.docker.com/editions/community/docker-ce-desktop-windows).

2. **Follow the Installation Wizard:**

   - Follow the instructions on the installation wizard screens. When prompted, ensure the option to use WSL 2 is selected.

3. **Start Docker Desktop:**

   - Once the installation is complete, start Docker Desktop from the Start menu.

4. **Complete the Setup:**
   - Follow the guided onboarding to complete the setup.

## Step 6: Verify Docker Installation

1. **Open PowerShell or Command Prompt:**

   - Open PowerShell or Command Prompt as an Administrator.

2. **Run the Test Command:**

   - Verify that Docker is installed correctly by running the following command:
     ```sh
     docker --version
     ```

3. **Run a Test Container:**

   - Run a test Docker container to ensure everything is set up correctly:
     ```sh
     docker run hello-world
     ```

   This command downloads a test image and runs it in a container. When the container runs, it prints a confirmation message.

## Step 7: Using Docker with WSL 2

1. **Set WSL 2 Integration:**

   - Open Docker Desktop, go to "Settings" -> "Resources" -> "WSL Integration". Ensure your installed WSL 2 distributions are enabled for integration with Docker.

2. **Access Docker from WSL 2:**
   - Open your WSL 2 terminal (e.g., Ubuntu) and run Docker commands directly.

## Troubleshooting

If you encounter any issues during the installation, here are some common troubleshooting steps:

- Ensure that the Docker Desktop service is running.
- Restart your computer to ensure all changes take effect.
- Check Docker logs for errors: Open Docker Desktop and go to "Troubleshoot" -> "Get Support" to view logs.
- Ensure WSL 2 is installed correctly and set as the default version.
