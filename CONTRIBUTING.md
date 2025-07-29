# Contributing to ROS2-X Container Boilerplate

Thank you for your interest in contributing! This boilerplate is community-driven, and your help improves it for everyone in the ROS2/AI/robotics space. Whether fixing bugs, adding sensors/ML stubs, improving docs, or suggesting features, all input is welcome.

## How to Contribute

1. **Fork the Repository**: Click "Fork" on GitHub to create your copy.
2. **Clone Locally**: `git clone https://github.com/yourusername/ros2-x-container.git` (replace with your fork).
3. **Create a Branch**: `git checkout -b feat/your-feature` or `fix/your-bug-fix` (use descriptive names).
4. **Make Changes**:
   - Follow PEP8 for Python code (use `black` or `flake8` for formatting/linting).
   - Add tests if modifying code (use pytest in src/stream2ros/tests if added).
   - Update README.md for new features (e.g., extensions usage).
   - Keep commits atomic and messages clear (e.g., "feat: add LiDAR sensor stub").
5. **Test Locally**: Rebuild Docker (`docker build -t orin-container .`), run with your changes, verify in Foxglove.
6. **Commit and Push**: `git commit -m "feat: add new sensor stub"` and `git push origin feat/your-feature`.
7. **Open a Pull Request**: Go to your fork on GitHub, click "Pull Request". Provide a clear title/description, reference issues if applicable.
8. **Review Process**: I'll review PRs promptly. Be open to feedback; we aim for high-quality merges.

## Code Style and Standards
- **Python**: PEP8, 4-space indentation. Use type hints where possible.
- **Docker**: Keep lightweight; add deps only if essential.
- **Extensions**: Follow stubs in extensions/sensors and /algorithms—keep modular.
- **Tests**: Add unit tests for new code (run with `colcon test`).

## Reporting Issues
- Use GitHub Issues for bugs/features.
- Provide: Steps to reproduce, expected vs actual behavior, screenshots/logs, environment (e.g., Jetson/Mac, ROS version).

## Questions?
Post in Discussions or Issues. Let's build something great!

Follow the [Code of Conduct](CODE_OF_CONDUCT.md) for a positive environment.