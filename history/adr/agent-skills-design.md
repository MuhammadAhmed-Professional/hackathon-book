# ADR-005: Agent Skills Design Using Persona + Questions + Principles Pattern

**Status**: Accepted  
**Date**: 2025-01-27  
**Context**: Need to implement reusable agent skills (SummarizeSection, GenerateQuizQuestions, ExplainTerm) that can be reused across future projects. Skills should follow a consistent pattern that captures reusable intelligence.

## Decision

We will implement agent skills using the **Claude Code Subagents pattern** with **Persona + Questions + Principles (P+Q+P)** structure.

## Rationale

### Persona + Questions + Principles Pattern

Each skill follows a consistent structure:

1. **Persona**: Defines the role and expertise of the agent
   - Example: "Educational Content Specialist" for SummarizeSection
   - Provides context for how the agent should behave

2. **Analytical Questions**: Questions the agent considers when processing input
   - Example: "What are the key points?", "What is the main message?"
   - Guides the agent's thinking process

3. **Decision Principles**: Rules the agent follows when making decisions
   - Example: "Conciseness: Summaries should be 20-30% of original length"
   - Ensures consistent, predictable behavior

### Benefits

1. **Reusability**: Skills can be easily reused in future projects
2. **Consistency**: All skills follow the same pattern
3. **Documentation**: P+Q+P structure serves as self-documentation
4. **Maintainability**: Clear structure makes skills easy to understand and modify
5. **Traceability**: Pattern links to Claude Code Subagents methodology

## Implementation

### Skill Structure

Each skill file contains:

```python
# Persona
PERSONA = "..."

# Analytical Questions
ANALYTICAL_QUESTIONS = [
    "Question 1",
    "Question 2",
    ...
]

# Decision Principles
DECISION_PRINCIPLES = [
    "Principle 1",
    "Principle 2",
    ...
]

def skill_function(input):
    # Uses PERSONA, ANALYTICAL_QUESTIONS, DECISION_PRINCIPLES
    # in prompt construction
    ...
```

### Skills Implemented

1. **SummarizeSection**: Summarizes text sections
   - Persona: Educational Content Specialist
   - Questions: Key points, main message, concepts to preserve
   - Principles: Conciseness, accuracy, educational value

2. **GenerateQuizQuestions**: Creates quiz questions
   - Persona: Educational Assessment Designer
   - Questions: Concepts to test, difficulty level, learning objectives
   - Principles: Clear questions, plausible distractors, one correct answer

3. **ExplainTerm**: Explains terms using context
   - Persona: Educational Content Explainer
   - Questions: Term definition, usage in context, examples
   - Principles: Simple language, book context, examples

## Consequences

**Positive**:
- Skills are reusable across projects
- Consistent pattern makes skills easy to understand
- Self-documenting through P+Q+P structure
- Aligns with Claude Code Subagents methodology
- Enables bonus marks for hackathon

**Negative**:
- Requires discipline to maintain pattern consistency
- Slightly more verbose than simple functions
- Need to document pattern for team understanding

## Alternatives Considered

- **Simple Functions**: No structure, just implementation
  - Rejected: Lacks reusability and documentation
- **Class-Based**: Skills as classes with methods
  - Rejected: Over-engineering for current needs
- **Generic P+Q+P**: Pattern without Claude Code Subagents alignment
  - Rejected: Doesn't qualify for bonus marks

## References

- Claude Code Subagents Documentation
- Spec-Kit Plus Reusable Intelligence Guidelines
- Hackathon Requirements for Bonus Marks

