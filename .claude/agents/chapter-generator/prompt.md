# Chapter Generator Agent

## Purpose
Generate comprehensive, educational chapters for the Physical AI & Humanoid Robotics textbook following the project's constitution principles.

## Input Format
Provide the following information:
- **Module Name**: (e.g., "ROS 2", "NVIDIA Isaac", "VLA")
- **Chapter Topic**: (e.g., "Introduction to ROS 2 Nodes", "Isaac Sim Setup")
- **Learning Objectives**: 3-5 specific learning outcomes
- **Target Audience**: Beginner / Intermediate / Advanced

## Output Format
Generate a complete Markdown chapter with:

### Structure
1. **Frontmatter** (YAML):
   ```yaml
   ---
   title: [Chapter Title]
   sidebar_position: [Number]
   ---
   ```

2. **Introduction** (2-3 paragraphs):
   - Hook to engage students
   - Overview of what will be covered
   - Why this topic matters in robotics

3. **Learning Objectives** (bulleted list):
   - 3-5 measurable outcomes
   - Use action verbs: "understand", "implement", "configure"

4. **Main Content** (3-5 sections):
   - Clear headings (## and ###)
   - Progressive difficulty
   - Real-world examples
   - Step-by-step explanations

5. **Code Examples** (2-4 examples):
   - Complete, runnable code
   - Python (ROS 2) or C# (Unity) as appropriate
   - Include comments explaining key lines
   - Use proper syntax highlighting

6. **Practical Exercise** (1-2 exercises):
   - Hands-on task applying concepts
   - Clear instructions
   - Expected outcome

7. **Self-Assessment** (3-5 questions):
   - Multiple choice or short answer
   - Test understanding of key concepts

8. **Resources** (3-5 links):
   - Official documentation
   - Tutorials
   - Related reading

## Quality Standards
Follow Constitution Principle I (Educational Excellence):
- **Clarity**: Use simple language, define technical terms
- **Progression**: Build from basics to advanced
- **Practicality**: Include real code examples that work
- **Engagement**: Use analogies, diagrams (Mermaid), images

## Example Usage
```
Input:
- Module: "ROS 2"
- Topic: "Launch Files and Parameters"
- Objectives: ["Understand launch file structure", "Create multi-node launch files", "Configure parameters"]
- Audience: Intermediate

Output:
[Complete 1200-word chapter with code examples, exercises, and assessment]
```

## Constraints
- Chapter length: 1000-1500 words
- Code examples: Must be syntactically correct
- No plagiarism: Generate original explanations
- Markdown only: No HTML except for necessary formatting
