# Installing Docker on Linux

This guide provides step-by-step instructions to install Docker on a Linux system, specifically focusing on Ubuntu. The steps may slightly vary for other distributions.

## Step 1: Update the Package Database

Open a terminal and update your existing list of packages:

```sh
sudo apt update
```

## Step 2: Install Required Packages

Install the packages that allow apt to use repositories over HTTPS:

```sh
sudo apt install apt-transport-https ca-certificates curl software-properties-common
```

## Step 3: Add Docker’s Official GPG Key

Add the GPG key for Docker’s official repository to your system:

```sh
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```

## Step 4: Add Docker’s APT Repository

Add the Docker repository to APT sources:

```sh
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
```

Update the package database with the Docker packages from the newly added repo:

```sh
sudo apt update
```

Ensure you are about to install from the Docker repo instead of the default Ubuntu repo:

```sh
apt-cache policy docker-ce
```

## Step 5: Install Docker

Install Docker:

```sh
sudo apt install docker-ce
```

Verify that Docker is installed and running:

```sh
sudo systemctl status docker
```

## Step 6: Manage Docker as a Non-Root User (Optional)

To avoid needing `sudo` for Docker commands, add your user to the `docker` group:

```sh
sudo usermod -aG docker ${USER}
```

Log out and log back in to apply the group change.

Alternatively, you can use the following command to refresh group membership without logging out:

```sh
su - ${USER}
```

Verify that your user is now added to the `docker` group:

```sh
groups ${USER}
```

## Step 7: Test Docker Installation

Verify the installation by running a test Docker container:

```sh
docker run hello-world
```

This command downloads a test image and runs it in a container. When the container runs, it prints a confirmation message.

## Troubleshooting

If you encounter any issues during the installation, here are some common troubleshooting steps:

- Ensure that the Docker service is running:

  ```sh
  sudo systemctl start docker
  ```

- Check Docker logs for errors:

  ```sh
  sudo journalctl -u docker.service
  ```

- Verify that your user is correctly added to the `docker` group.
