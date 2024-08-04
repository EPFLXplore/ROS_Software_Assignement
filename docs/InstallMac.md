# Installing Docker on macOS

This guide provides step-by-step instructions to install Docker on a macOS system.

## Step 1: Check System Requirements

Ensure your system meets the following requirements:

- macOS must be version 10.14 or newer.
- You have administrative privileges on your macOS system.

## Step 2: Download Docker Desktop for Mac

1. **Download Docker Desktop:**
   - Go to the [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop) page and click "Download for Mac".

## Step 3: Install Docker Desktop

1. **Run the Installer:**

   - Open the downloaded `.dmg` file.

2. **Drag Docker to Applications:**

   - In the installation window, drag the Docker icon to the Applications folder.

3. **Start Docker Desktop:**

   - Go to the Applications folder and double-click Docker to start Docker Desktop.

4. **Complete the Setup:**
   - You might be prompted to enter your password to install a helper tool. Enter your password and proceed with the installation.
   - Follow the guided onboarding to complete the setup.

## Step 4: Verify Docker Installation

1. **Open Terminal:**

   - Open the Terminal application from the Applications folder or using Spotlight Search.

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

## Step 5: Configure Docker Desktop

1. **Open Docker Desktop Preferences:**

   - Click on the Docker icon in the menu bar and select "Preferences".

2. **Configure Resources:**
   - Under the "Resources" tab, you can allocate CPU, memory, and disk resources to Docker. Adjust these settings based on your system's capabilities and your workload requirements.

## Step 6: Using Docker

1. **Open Terminal:**

   - Open the Terminal application.

2. **Run Docker Commands:**
   - You can now run Docker commands directly from the Terminal. For example, to list all running containers, use:
     ```sh
     docker ps
     ```

## Troubleshooting

If you encounter any issues during the installation, here are some common troubleshooting steps:

- Ensure that the Docker Desktop application is running.
- Restart your computer to ensure all changes take effect.
- Check Docker logs for errors: Click on the Docker icon in the menu bar, select "Troubleshoot", and then "Get Support" to view logs.
- Ensure your macOS is updated to the latest version.
