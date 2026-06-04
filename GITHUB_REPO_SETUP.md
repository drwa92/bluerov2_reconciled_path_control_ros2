# GitHub repository and GitHub Pages setup

This file gives the exact commands to publish the package to GitHub and enable the documentation website.

## 1. Choose repository name

Recommended repository name:

```text
bluerov2_reconciled_path_control_ros2
```

Recommended repository description:

```text
ROS 2 direct-thruster BlueROV2 path-following controller for DAVE/Gazebo with go-to, waypoint, circle, spiral, emergency-stop, and optional virtual-wrench reconciliation.
```

## 2. Initialize git locally

From this repository root:

```bash
git init
git branch -M main
git add .
git commit -m "Initial release: BlueROV2 path control with optional reconciliation"
```

## 3. Create the repository with GitHub CLI

Install and authenticate GitHub CLI if needed:

```bash
gh auth login
```

Create a public repository and push:

```bash
gh repo create drwa92/bluerov2_reconciled_path_control_ros2 \
  --public \
  --description "ROS 2 direct-thruster BlueROV2 path-following controller for DAVE/Gazebo with optional virtual-wrench reconciliation." \
  --source=. \
  --remote=origin \
  --push
```

If you prefer to create the repository from the GitHub web interface, create an empty public repository, then run:

```bash
git remote add origin https://github.com/drwa92/bluerov2_reconciled_path_control_ros2.git
git push -u origin main
```

## 4. Enable GitHub Pages

This repository includes a static website in:

```text
docs/
```

In GitHub:

1. Open the repository.
2. Go to **Settings**.
3. Go to **Pages**.
4. Under **Build and deployment**, select **Deploy from a branch**.
5. Select branch **main** and folder **/docs**.
6. Save.

The website will be published at:

```text
https://drwa92.github.io/bluerov2_reconciled_path_control_ros2/
```

## 5. Repository metadata

This scaffold is already personalized for `drwa92`, Waseem Akram, and MARVIS LAB. Update `README.md`, `docs/`, and `CITATION.cff` only if the repository name or maintainer details change.

## 6. First release

After testing the package in DAVE:

```bash
git tag v0.1.0
git push origin v0.1.0
```

Then create a GitHub Release from the tag.
