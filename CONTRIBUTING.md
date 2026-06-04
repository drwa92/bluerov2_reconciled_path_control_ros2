# Contributing

Thank you for considering a contribution.

## Development workflow

1. Fork the repository.
2. Create a feature branch.
3. Test in DAVE/Gazebo using direct-control mode.
4. Open a pull request with a clear description.

## Code style

- Keep ROS nodes small and readable.
- Prefer explicit parameters over hard-coded constants.
- Do not commit generated build, install, log, bag, or cache files.
- Keep research-only fault-injection experiments separate from the clean controller package unless the feature is generally useful.

## Safety

Any controller or actuator-allocation change must be tested in simulation before hardware use. Include the DAVE launch command and service calls used for validation in the pull request.
