# Validate Chapter Structure Skill

**Purpose**: Validate textbook chapters against quality standards, checking for completeness, technical accuracy, and pedagogical effectiveness.

**When to Use**: After writing a chapter, before committing to repository.

---

## Validation Checklist

### 1. Frontmatter Validation
```yaml
✅ id: kebab-case, unique
✅ title: Clear, descriptive (max 60 chars)
✅ sidebar_label: Short version (max 25 chars)
✅ sidebar_position: Correct sequence number
✅ description: One sentence, ends with period
✅ tags: 3-7 relevant tags
```

### 2. Structure Validation
- ✅ Introduction with learning objectives (3-5 bullets)
- ✅ Prerequisites checklist
- ✅ Estimated completion time
- ✅ Conceptual overview (300-500 words)
- ✅ Hands-on sections (2-3 minimum)
- ✅ Code examples (working, with comments)
- ✅ Exercises (3-5, tiered by difficulty)
- ✅ Summary with key takeaways
- ✅ References to official docs

### 3. Code Quality Validation
- ✅ All imports included
- ✅ Error handling present
- ✅ Comments explain non-obvious logic
- ✅ Realistic variable names
- ✅ Code blocks have language tags
- ✅ Expected outputs shown
- ✅ Test commands provided

### 4. Pedagogical Validation
- ✅ Progressive complexity (simple → advanced)
- ✅ Real-world applications mentioned
- ✅ Analogies for complex concepts
- ✅ Common mistakes documented
- ✅ Troubleshooting sections
- ✅ Bloom's Taxonomy coverage

### 5. Technical Accuracy
- ✅ Correct ROS 2 message types
- ✅ Valid URDF/SDF syntax
- ✅ Accurate hardware specifications
- ✅ Correct command syntax
- ✅ Links to official documentation

### 6. Length Validation
- ✅ Total: 2000-3000 words (5-10 min read)
- ✅ Code-to-text ratio: 30-40% code
- ✅ Not too dense (max 500 words per section)

---

## Validation Output Format

```markdown
## Chapter Validation Report

**Chapter**: [Chapter Title]
**File**: [File path]

### ✅ Passed (12/15)
- Frontmatter complete
- Learning objectives present
- Code examples working
- ... [list all passed items]

### ⚠️ Warnings (2)
- Code block at line 145 missing language tag
- Exercise difficulty not balanced (4 easy, 0 hard)

### ❌ Failed (1)
- Missing prerequisites checklist
- Summary section < 100 words

### Recommendations
1. Add `python` language tag to code block at line 145
2. Convert 2 easy exercises to hard challenges
3. Expand summary to include next steps

**Overall Score**: 80/100 (Good - Minor improvements needed)
```
