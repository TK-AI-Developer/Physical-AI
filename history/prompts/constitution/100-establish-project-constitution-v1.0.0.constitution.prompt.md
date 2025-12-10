---
id: 100
title: Establish Project Constitution v1.0.0
stage: constitution
date: 2025-12-10
surface: agent
model: gemini
feature: none
branch: master
user: {{USER}}
command: /sp.constitution
labels: [constitution, documentation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:
[]
---

## Prompt

$ARGUMENTS
---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.constitution  Project: AI/Spec-Driven Book on Physical AI & Humanoid Robotics
Platform: Docusaurus → GitHub Pages
Tools: Spec-Kit Plus + Claude Code

Core Principles:
– Educational clarity (beginner → advanced)
– Technical accuracy (AI, robotics, cloud-native)
– Practical outcomes (hands-on examples)
– Ethical responsibility (safety & responsible AI)
Standards:

– Content must be original and source–traceable
– Code examples must run successfully
– Robotics concepts must align with ROS2, URDF, and real control systems
– Agentic AI content must reflect current production practices
– Citations: official docs or reputable research
– Tone: mentor → student, respectful, direct

Structure:

– Each chapter starts from a specification
– Include objectives, examples, steps, and a final exercise

– Every technical module includes:
 • Inputs, outputs, architecture, code, failure modes, safety notes

Constraints:

– Minimum total length: 20,000–35,000 words
– At least 1 working demo (simulation acceptable)
– Images/figures must include alt text

Success Criteria:

– A student can follow and produce a working robotics or AI system
– Book deploys to GitHub Pages with no build errors
– All code reproducible and free of hallucinations

End of constitution

## Response snapshot

I have successfully updated the project constitution to version 1.0.0.

The new constitution is located at `.specify/memory/constitution.md`.

I also updated the `.specify/templates/plan-template.md` to reflect the new principles in its "Constitution Check" section.

A Prompt History Record has been created at `history/prompts/constitution/100-establish-project-constitution-v1.0.0.constitution.prompt.md`.

Suggested commit message: `docs: establish constitution v1.0.0`

## Outcome

- ✅ Impact: {{OUTCOME_IMPACT}}
- 🧪 Tests: {{TESTS_SUMMARY}}
- 📁 Files: {{FILES_SUMMARY}}
- 🔁 Next prompts: {{NEXT_PROMPTS}}
- 🧠 Reflection: {{REFLECTION_NOTE}}

## Evaluation notes (flywheel)

- Failure modes observed: {{FAILURE_MODES}}
- Graders run and results (PASS/FAIL): {{GRADER_RESULTS}}
- Prompt variant (if applicable): {{PROMPT_VARIANT_ID}}
- Next experiment (smallest change to try): {{NEXT_EXPERIMENT}}
