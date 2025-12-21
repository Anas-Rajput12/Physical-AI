# Data Model: Physical AI & Humanoid Robotics Book

## Course Content Entity
- **Name**: Course Content
- **Description**: Educational material for the Physical AI & Humanoid Robotics course
- **Fields**:
  - title: string (chapter/module title)
  - content: Markdown text (main content)
  - concepts: array of concept objects (key concepts covered)
  - architectureDiagrams: array of diagram objects (visual representations)
  - examples: array of example objects (hands-on examples)
  - exercises: array of exercise objects (mini exercises)
  - learningObjectives: array of strings (what students should learn)
  - prerequisites: array of strings (required knowledge)
  - estimatedTime: number (minutes to complete)

## Concept Entity
- **Name**: Concept
- **Description**: A key concept within a chapter
- **Fields**:
  - id: string (unique identifier)
  - name: string (concept name)
  - description: string (detailed explanation)
  - relatedConcepts: array of concept IDs (connections to other concepts)
  - examples: array of example references (examples demonstrating this concept)

## Architecture Diagram Entity
- **Name**: Architecture Diagram
- **Description**: Visual representation of system architecture
- **Fields**:
  - id: string (unique identifier)
  - title: string (diagram title)
  - description: string (what the diagram illustrates)
  - content: string (Mermaid or ASCII diagram code)
  - relatedChapter: string (chapter this diagram belongs to)
  - tags: array of strings (technology/components shown)

## Example Entity
- **Name**: Example
- **Description**: Hands-on example for students to follow
- **Fields**:
  - id: string (unique identifier)
  - title: string (example title)
  - description: string (what the example demonstrates)
  - code: string (code snippet or commands)
  - language: string (programming language or tool)
  - difficulty: enum (beginner, intermediate, advanced)
  - relatedConcepts: array of concept IDs (concepts demonstrated)

## Exercise Entity
- **Name**: Exercise
- **Description**: Mini exercise for students to practice
- **Fields**:
  - id: string (unique identifier)
  - title: string (exercise title)
  - description: string (what the student needs to do)
  - difficulty: enum (beginner, intermediate, advanced)
  - expectedOutcome: string (what the solution should achieve)
  - hints: array of strings (help for students)
  - solution: string (reference solution)
  - relatedConcepts: array of concept IDs (concepts practiced)

## Chapter Entity
- **Name**: Chapter
- **Description**: A complete chapter in the book
- **Fields**:
  - id: string (unique identifier like "ros2", "digital-twin", etc.)
  - title: string (chapter title)
  - content: Course Content reference (the main content)
  - order: number (sequence in the book)
  - dependencies: array of chapter IDs (chapters that should be completed first)
  - learningObjectives: array of strings (what students will learn)
  - estimatedCompletionTime: number (minutes to complete chapter)