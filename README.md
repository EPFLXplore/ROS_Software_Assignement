# Xplore Rover Challenge - ROS Humble Assignment

Welcome to the XRC ROS Humble Stack assignment!

This assignment is designed to help you get acquainted with ROS (Robot Operating System) using Docker, while building foundational skills that are crucial for developing a rover.

> [!NOTE]
>
> From level 2 onwards, both paths (`a` and `b`) are not mandatory.
>
> You can choose whether you want to do the Python or C++ one, or do both to see the differences and help you choose for your rover infrastructure.

## Table of Contents

- [Introduction](#introduction)
- [Assignment Levels](#assignment-levels)
  - [Level 1: Introduction and Setup](#level-1-introduction-and-setup)
  - [Level 2: Package Creation and Communication](#level-2-package-creation-and-communication)
  - [Level 3: Package Infrastructure and Multiple Nodes](#level-3-package-infrastructure-and-multiple-nodes)
- [Useful Resources](#useful-resources)
- [Useful Commands](#useful-commands)

## Introduction

In this assignment, you will:

1. Set up your environment with Docker and ROS Humble (and understand what the hell they are?!).
2. Create ROS packages and understand the basics of publisher-subscriber communication.
3. Develop a custom ROS infrastructure with multiple publishers and subscribers

By the end of this assignment, you should have a solid understanding of ROS basics and how to use Docker.

## Assignment Levels

### Level 1: Introduction and Setup

**Objectives:**

- Install Docker;
- Install ROS Humble using Docker;
- Test a built-in ROS command;
- Understand the concept of ROS and its advantages.

Detailed instructions for Level 1 can be found in the [Level 1 file](./docs/levels/Level1.md).

### Level 2: Package Creation and Communication

**Objectives:**

- Create a ROS package inside the Docker image;
- Implement a simple publisher and subscriber.

Detailed instructions for Level 2 can be found

- in the [Level 2 Python file](./docs/levels/py/Level2.md).
- in the [Level 2 C++ file](./docs/levels/cpp/Level2.md).

### Level 3: Package Infrastructure and Multiple Nodes

**Objectives:**

- Take the pub/sub written in Level 2 and extend it to a multi-node infrastructure.

Detailed instructions for Level 3 can be found

- in the [Level 3 Python file](./docs/levels/py/Level3.md).
- in the [Level 3 C++ file](./docs/levels/cpp/Level3.md).

## Installation

You will need to fork this repository, and install Docker. Follow the procedure for your system:

- [Windows](./docs/InstallWindows.md)
- [Mac](./docs/InstallMac.md)
- [Linux (mostly Ubuntu)](./docs/InstallLinux.md)

> [!NOTE]
>
> Please use a UNIX-based system (aka Mac or Linux) to avoid A LOT of problems.

## Troubleshooting

If you have any trouble or problem with the assignment, do not hesitate to ask question to the coaches.

If you find a problem or an error in the assignment, feel free to create an issue: [+ Create Issue](https://github.com/EPFLXplore/ROS_Software_Assignement/issues/new/choose)

## Useful Resources

Here are some resources to help you throughout the assignment:

- ROS 2
  - [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
  - [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
  - [ROS 2 Youtube Tutorials](https://www.youtube.com/watch?v=0aPbWsyENA8&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy)
- Docker
  - [Docker Documentation](https://docs.docker.com/)

## Useful Commands

We have compiled a list of useful Linux, Docker, and ROS commands to assist you with the assignment. These can be found in the [`UsefulCommands.md`](./docs/UsefulCommands.md) file.

Good luck, and don't forget to have fun!

![GIF](https://media2.giphy.com/media/v1.Y2lkPTc5MGI3NjExZTZib3ZmdWVtdGNyOGxtaWw2aHVwcDd5OTI3bXM1ZWZpNzczM3ZjMSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/VbnUQpnihPSIgIXuZv/giphy.gif)
