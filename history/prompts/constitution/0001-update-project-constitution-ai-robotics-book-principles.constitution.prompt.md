---
id: 0001
title: Update project constitution with AI/Robotics book principles
stage: constitution
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
null
---

## Prompt

Project: AI/Spec-Driven Book Creation for Physical AI & Humanoid Robotics using Docusaurus, Spec-Kit Plus, and Claude Code, deployed on GitHub Pages.

Core Principles:
- Technical Accuracy: All AI, robotics, hardware, and software concepts must come from primary, reputable sources.
- Clarity for Learners: Writing must be simple, beginner-friendly, and understandable for students and new developers.
- AI-Native Authoring: All work must follow Spec-Kit Plus + Claude Code loops for drafting, refining, and verifying content.
- Consistency & Modularity: All chapters follow one template and structure for unified reading.
- Practical Learning: Include real examples, diagrams, mini-projects, exercises, and hands-on activities.
- Ethical & Safe AI/Robotics: Safety rules and responsible AI usage must be included throughout the book.

Key Standards:
- Sources:
  - 40% academic/research lab sources (MIT, CMU, Stanford, IEEE, etc.)
  - 40% official documentation (ROS, NVIDIA, Boston Dynamics, OpenAI, Arduino, ISO standards)
  - Remaining from verified blogs, textbooks, and technical guides
- Citation Style: APA or MLA depending on chapter type
- Plagiarism: 0% tolerance; all content must be original
- Writing Level: Flesch-Kincaid Grade 7–10 for clarity
- Code Standards:
  - All code must be explained
  - Code must be runnable or simulator-compatible (Python, ROS, Webots, Arduino)
- Visual Standards:
  - Only original or AI-generated diagrams allowed
  - Every diagram must be labeled and explained
- Formatting:
  - Use Docusaurus Markdown with proper headings, bullets, and callouts

Constraints:
- Chapter Count: 8–12 chapters
- Word Count: 20,000–35,000 words total
- Each chapter must include: Overview, Concepts, Diagrams, Examples/Code, Mini Exercise, Summary, Suggested Readings
- Tools Required: Docusaurus + GitHub Pages deployment
- Workflow: Must follow full Spec-Kit Plus flow (constitution → scope → workflow → tasks → drafts)
- Deliverables: Complete Docusaurus website + GitHub Pages deployment

Success Criteria:
- All claims verified with reliable sources
- All chapters follow the same template and readability standards
- All code runs correctly or simulates properly
- Docusaurus build passes with no errors
- Final book is cohesive, clean, professional, and easy to learn from
- Successfully deployed public website on GitHub Pages
- Complete Spec-Kit Plus process followed end to end.

## Response snapshot

The project constitution has been updated to version 0.1.0, incorporating detailed principles, standards, constraints, and success criteria for the AI/Spec-Driven Book Creation project. Several template files were flagged for manual follow-up to ensure consistency.

## Outcome

- ✅ Impact: Project constitution updated and aligned with user requirements.
- 🧪 Tests: null
- 📁 Files: .specify/memory/constitution.md modified
- 🔁 Next prompts: Review flagged files for consistency.
- 🧠 Reflection: Successfully updated the constitution and identified downstream impacts.

## Evaluation notes (flywheel)

- Failure modes observed: Failed to execute powershell script, switched to agent-native tools.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Improve powershell script execution reliability.
