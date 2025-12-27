# Physical AI & Humanoid Robotics Course

This Docusaurus project contains the complete documentation for the "Physical AI & Humanoid Robotics" course designed for senior undergraduate AI and robotics students.

## Course Overview

This comprehensive course covers:

- **Physical AI and Embodied Intelligence**: Understanding AI systems that interact with the physical world
- **ROS 2 Robotic Nervous System**: Communication framework for robotic platforms
- **Digital Twin Simulation**: Gazebo and Unity for robot development and testing
- **AI-Robot Brain**: NVIDIA Isaac for intelligent robot behaviors
- **Vision-Language-Action (VLA) Systems**: Connecting perception, language, and action
- **Capstone Project**: Autonomous humanoid system integrating all concepts

## Installation

1. Make sure you have Node.js version 18 or higher installed
2. Install dependencies:
   ```bash
   npm install
   ```
3. Start the development server:
   ```bash
   npm start
   ```

## Building the Documentation

To build the static files for deployment:

```bash
npm run build
```

## Local Development

```bash
# Start local development server
npm start

# Open http://localhost:3000 to view the documentation
```

## Deployment

This project is set up for deployment to GitHub Pages. To deploy:

```bash
npm run deploy
```

## Project Structure

```
my-physical-ai-book/
├── docs/                 # All course content
│   ├── intro.md          # Course introduction
│   ├── ros2/            # ROS 2 module
│   ├── digital-twin/    # Digital twin module
│   ├── ai-brain/        # AI-robot brain module
│   ├── vla/             # VLA module
│   └── capstone/        # Capstone project
├── src/                 # Custom React components and CSS
├── static/              # Static assets (images, etc.)
├── docusaurus.config.js # Docusaurus configuration
├── sidebars.js          # Navigation sidebar configuration
└── package.json         # Dependencies and scripts
```

## Contributing

Feel free to contribute to this course documentation by:

1. Forking the repository
2. Creating a feature branch
3. Making your changes
4. Submitting a pull request

## License

This educational content is provided for academic purposes.