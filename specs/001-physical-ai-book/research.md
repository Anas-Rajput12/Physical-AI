# Research: Physical AI & Humanoid Robotics Book

## Decision: Docusaurus Setup and Configuration
**Rationale**: Docusaurus is the optimal static site generator for documentation and book projects, offering features like versioning, search, and multi-language support. It's specifically designed for technical documentation and will provide the best user experience for students.
**Alternatives considered**:
- GitBook: Limited customization options compared to Docusaurus
- Hugo: More complex configuration for this use case
- Custom React app: More development overhead without added benefits

## Decision: GitHub Pages Deployment Strategy
**Rationale**: GitHub Pages provides free hosting with reliable uptime, seamless integration with Git workflows, and custom domain support. It's ideal for educational content that needs to be accessible to students without infrastructure costs.
**Alternatives considered**:
- Netlify: Requires additional configuration for similar functionality
- Vercel: More complex for static documentation sites
- Self-hosting: Unnecessary complexity and costs for educational content

## Decision: Claude Code Integration Approach
**Rationale**: Claude Code will be used to assist with content creation, technical accuracy verification, and maintaining consistent formatting standards. It provides AI-powered assistance while maintaining human oversight for educational quality.
**Alternatives considered**:
- Manual writing only: More time-consuming without AI assistance
- Different AI tools: Claude Code is specifically designed for development workflows

## Decision: Markdown Standards for Educational Content
**Rationale**: Standard Docusaurus Markdown with additional educational extensions will ensure compatibility while providing features needed for educational content like exercises, diagrams, and interactive elements.
**Alternatives considered**:
- RestructuredText: Less familiar to developers and students
- AsciiDoc: More complex syntax for the same functionality

## Decision: Folder Structure for Docusaurus Book
**Rationale**: Standard Docusaurus structure with dedicated sections for each chapter, exercises, and supplementary materials provides clear organization and easy navigation for students.
**Alternatives considered**:
- Flat structure: Would be harder to navigate for a multi-chapter book
- Custom structure: Would break standard Docusaurus conventions

## Decision: Chapter Writing Order
**Rationale**: Following the priority order from the specification (P1, P2, P3) ensures foundational concepts are established before advanced topics. Introduction first, then ROS 2, Digital Twin, AI-Robot Brain, VLA, and finally the capstone project.
**Alternatives considered**:
- Alphabetical order: Would not follow logical learning progression
- Reverse order: Would confuse students without foundational knowledge