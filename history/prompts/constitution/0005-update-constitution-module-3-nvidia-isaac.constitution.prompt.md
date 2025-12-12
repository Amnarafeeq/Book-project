---
id: 0005
title: Update constitution for Module 3 - NVIDIA Isaac
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "module-3"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md modified
tests:
null
---

## Prompt

Project: Module 3 – NVIDIA Isaac (Sim + ROS + VSLAM)

Core Principles:
- Precision with Isaac terminology
- Practical robotics AI explanations
- Connect Isaac → ROS2 → Humanoid movement
- Use visual mental models

Key Standards:
- Explain Omniverse, USD, Isaac Sim UI
- Cover perception, mapping, navigation
- Demonstrate Isaac ROS packages (Stereo, VSLAM, Depth)
- Include GPU concepts simply (no CUDA programming)

Constraints:
- No advanced linear algebra
- Avoid low-level rendering pipeline details
- Keep focus on humanoids, not industrial robots

Success Criteria:
- Reader can open Isaac Sim, spawn robot, use sensors
- Reader understands Nav2 navigation pipeline
- Reader understands Isaac ROS accelerated perception

## Response snapshot

The project constitution for "Module 3 – NVIDIA Isaac (Sim + ROS + VSLAM)" has been updated to version 0.5.0. This update includes detailed principles, standards, constraints, and success criteria specific to Module 3, with a minor version bump.

## Outcome

- ✅ Impact: Project constitution updated with specific guidelines for Module 3.
- 🧪 Tests: null
- 📁 Files: .specify/memory/constitution.md modified
- 🔁 Next prompts: Review flagged files for consistency.
- 🧠 Reflection: Successfully updated the constitution for Module 3 and identified downstream impacts.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Automate consistency checks for templates.
