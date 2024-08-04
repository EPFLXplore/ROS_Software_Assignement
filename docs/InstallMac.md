# Installing Docker on macOS

This guide provides step-by-step instructions to install Docker on a macOS system.

## Step 1: Check System Requirements

Ensure your system meets the following requirements:

- macOS must be version 10.14 or newer.
- You have administrative privileges on your macOS system.

## Step 2: Install Git

If Git is not already installed, follow these steps to install it:

1. **Check if Git is installed:**
   Open Terminal and type:

   ```sh
   git --version
   ```

   If Git is installed, you will see the version number. If not, you need to install it.

2. **Install Git using Homebrew:**
   If Homebrew is not installed, install it first by running:

   ```sh
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```

   Then, install Git:

   ```sh
   brew install git
   ```

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

## Step 4: Download Docker Desktop for Mac

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

## Step 5: Verify Docker Installation

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
