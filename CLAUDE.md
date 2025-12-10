# Claude Code Rules - Physical AI Textbook Platform

## Project Mission

You are working on an **AI-native educational platform** for teaching Physical AI & Humanoid Robotics. This project is a comprehensive hackathon submission combining cutting-edge educational technology with AI enhancement.

**Project Components:**
- **Educational Content**: Docusaurus-based interactive textbook covering ROS 2, Gazebo, NVIDIA Isaac, and VLA (Vision-Language-Action)
- **AI Enhancement**: RAG chatbot for intelligent Q&A using OpenAI + Qdrant Cloud
- **Personalization**: User profiles with hardware/software background for adaptive content
- **Localization**: English-to-Urdu translation with RTL support and technical term preservation
- **Modern Auth**: better-auth.com with onboarding questionnaire

**Your Success is Measured By:**
- Educational content is accurate, pedagogically sound, and technically precise
- RAG chatbot provides contextually relevant answers with proper source citations
- Selected-text mode works seamlessly for targeted question-answering
- Authentication and personalization enhance the learning experience
- Translation maintains technical accuracy while being accessible
- All features integrate smoothly with Docusaurus and GitHub Pages deployment
- Prompt History Records (PHRs) are created accurately for every user prompt
- Architectural Decision Records (ADRs) are suggested intelligently for significant decisions

---

## Core Technology Stack

### Frontend (Docusaurus + React)
- **Framework**: Docusaurus v3.x (Static Site Generator with React)
- **UI Library**: React 18+ with TypeScript
- **Styling**: CSS Modules + Docusaurus theming system
- **Deployment**: GitHub Pages (static build in `/docs` directory)
- **Key Files**:
  - `frontend/docusaurus.config.js` - Site configuration, baseUrl, plugins
  - `frontend/sidebars.js` - Navigation structure for modules
  - `frontend/docs/**/*.md` - Educational content (markdown/MDX)
  - `frontend/src/components/**` - React components (ChatBot, AuthUI, TranslationButton)
  - `frontend/src/theme/**` - Docusaurus theme overrides

### Backend (FastAPI + Python)
- **Framework**: FastAPI (Python 3.11+) with async support
- **Vector DB**: Qdrant Cloud (free tier) for document embeddings and semantic search
- **Database**: Neon Serverless Postgres for user profiles, chat history, translations cache
- **AI Services**:
  - OpenAI `text-embedding-3-small` for embeddings
  - OpenAI `gpt-4o-mini` for chat and translation
- **Authentication**: better-auth.com with custom user profile schema
- **Key Directories**:
  - `backend/src/api/` - FastAPI routers and endpoints
  - `backend/src/services/` - Business logic (RAG, translation, personalization)
  - `backend/src/models/` - Pydantic models & database schemas
  - `backend/src/db/` - Database connection and migrations

### Development Workflow (Spec-Driven Development)
- **Templates**: `.specify/templates/` - PHR, ADR, and planning templates
- **Specifications**: `specs/<feature>/spec.md` - Feature requirements
- **Architecture**: `specs/<feature>/plan.md` - Implementation plans and decisions
- **Tasks**: `specs/<feature>/tasks.md` - Testable task breakdown
- **History**:
  - `history/prompts/` - Prompt History Records (PHRs)
  - `history/adr/` - Architecture Decision Records (ADRs)

---

## Project-Specific Development Guidelines

### 1. Educational Content Standards (EXPANDED)

**When working with educational content (`frontend/docs/**/*.md`):**

#### Content Accuracy Requirements
- **Technical Precision**: All information about ROS 2, Gazebo, Isaac Sim, URDF, SDF must be factually correct and up-to-date
- **Version Specificity**: Clearly state versions (e.g., "ROS 2 Humble", "Gazebo Fortress", "Isaac Sim 4.0")
- **Command Accuracy**: Test all command-line instructions before publishing
- **Code Validation**: Every code example must be syntactically correct and executable
- **Hardware Realism**: Acknowledge hardware constraints (RTX 4080 vs. Jetson Orin Nano vs. budget laptops)

#### Pedagogical Structure
Each module/chapter should follow this progression:

1. **Learning Objectives** (3-5 concrete, measurable goals)
   ```markdown
   ## Learning Objectives
   By the end of this module, you will be able to:
   - Create a ROS 2 package with publisher and subscriber nodes
   - Define robot geometry using URDF with proper joint definitions
   - Launch a Gazebo simulation with sensor plugins
   ```

2. **Prerequisites** (What students need to know first)
   ```markdown
   ## Prerequisites
   - Basic Python programming (functions, classes, async/await)
   - Command-line familiarity (cd, ls, pip install)
   - Completed Module 1: ROS 2 Basics
   ```

3. **Conceptual Foundation** (Why this matters, real-world context)
   ```markdown
   ## Why ROS 2 Nodes Matter
   In robotics, we need independent processes that can:
   - Run at different frequencies (sensors at 100Hz, planning at 10Hz)
   - Fail independently without crashing the entire system
   - Be developed and tested in isolation
   ```

4. **Core Content with Progressive Complexity**
   - Start with simplest working example
   - Add complexity incrementally (basic → intermediate → advanced)
   - Explain each new concept before using it in code

5. **Hands-On Exercise** (Step-by-step tutorial)
   ```markdown
   ## Exercise: Create Your First ROS 2 Node

   ### Step 1: Set up the workspace
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_first_node
   ```

   ### Step 2: Write the publisher code
   ```python
   # my_first_node/my_first_node/publisher.py
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class MinimalPublisher(Node):
       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'topic', 10)
           self.timer = self.create_timer(0.5, self.timer_callback)

       def timer_callback(self):
           msg = String()
           msg.data = 'Hello ROS 2!'
           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing: "{msg.data}"')

   def main(args=None):
       rclpy.init(args=args)
       node = MinimalPublisher()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()
   ```
   ```

6. **Common Pitfalls & Debugging**
   ```markdown
   ## Common Issues

   ### "Package not found" error
   **Cause**: Workspace not sourced
   **Solution**: `source ~/ros2_ws/install/setup.bash`

   ### Node starts but no messages appear
   **Cause**: Publisher/subscriber topic mismatch
   **Solution**: Use `ros2 topic list` to verify topic names
   ```

7. **Hardware-Specific Guidance**
   ```markdown
   ## Hardware Considerations

   ### For High-End Workstations (RTX 4080+)
   - Run Isaac Sim for photorealistic rendering
   - Use synthetic data generation for training
   - Multiple parallel simulations possible

   ### For Budget Hardware (Jetson Orin Nano)
   - Use Gazebo Classic (lighter weight)
   - Focus on inference, not training
   - Cloud-based training, local deployment

   ### For Standard Laptops
   - Use ROS 2 without simulation for core concepts
   - Cloud-based simulation with AWS RoboMaker
   - Focus on code structure and algorithms
   ```

8. **Assessment Checkpoints**
   ```markdown
   ## Self-Assessment
   - [ ] I can create a ROS 2 package from scratch
   - [ ] I understand the difference between topics and services
   - [ ] I can visualize ROS 2 nodes with rqt_graph
   - [ ] I can debug using ros2 topic echo and ros2 node info
   ```

#### Code Example Standards
- **Language Specification**: Always use fenced code blocks with language
  ```python
  # Python example
  ```
  ```bash
  # Terminal commands
  ```
  ```xml
  <!-- URDF/SDF examples -->
  ```
- **Inline Comments**: Explain non-obvious logic
- **Complete Examples**: Include all imports and setup code
- **Output Examples**: Show expected terminal output
  ```bash
  $ ros2 topic list
  /parameter_events
  /rosout
  /topic
  ```

#### Visual Aid Requirements
- **Architecture Diagrams**: Use Mermaid for system architectures
  ```mermaid
  graph LR
    A[Camera Node] -->|Image| B[Vision Node]
    B -->|Detection| C[Planning Node]
    C -->|Commands| D[Motor Controller]
  ```
- **File Structure Diagrams**: For workspace layouts
- **Flowcharts**: For decision logic and algorithms

### 2. RAG Chatbot Development (EXPANDED)

**When modifying `backend/src/services/rag_service.py`:**

#### Embedding and Indexing Strategy

**Document Ingestion Pipeline:**
```python
# Chunking Strategy
CHUNK_SIZE = 500  # words
CHUNK_OVERLAP = 50  # words
METADATA_FIELDS = ["module", "chapter", "section", "file_path", "line_numbers"]

# Embedding Configuration
EMBEDDING_MODEL = "text-embedding-3-small"
EMBEDDING_DIMENSIONS = 1536
BATCH_SIZE = 100  # documents per batch to avoid rate limits
```

**Qdrant Collection Schema:**
```python
{
    "collection_name": "physical_ai_textbook",
    "vectors": {
        "size": 1536,
        "distance": "Cosine"  # For semantic similarity
    },
    "payload_schema": {
        "content": "text",  # The actual text chunk
        "module": "keyword",  # Module 1, Module 2, etc.
        "chapter": "keyword",  # Chapter name
        "section": "keyword",  # Section within chapter
        "file_path": "keyword",  # Source markdown file
        "line_start": "integer",
        "line_end": "integer",
        "code_blocks": ["text"],  # Extracted code snippets
        "hardware_level": "keyword"  # high-end, budget, basic
    }
}
```

#### Query Processing Rules

**1. Selected-Text Mode (Priority 1)**
```python
async def query_with_selected_text(
    question: str,
    selected_text: str,
    max_context_tokens: int = 3000
):
    """
    When user selects text and asks a question:
    1. ONLY use the selected text as context
    2. Do not retrieve additional chunks from vector DB
    3. Cite the specific selection in the response
    """
    # Embed the question for relevance scoring
    question_embedding = await embed_text(question)

    # Use selected text directly as context
    context = f"User-selected text:\n{selected_text}"

    # Generate answer with explicit source attribution
    response = await generate_answer(
        question=question,
        context=context,
        system_prompt="Answer based ONLY on the provided selected text. "
                     "Do not use external knowledge. "
                     "If the selection doesn't contain the answer, say so explicitly."
    )

    return {
        "answer": response,
        "sources": [{"type": "selected_text", "content": selected_text[:200]}],
        "mode": "selected_text"
    }
```

**2. Full-Context RAG Mode (General Questions)**
```python
async def query_full_context(
    question: str,
    user_profile: Optional[UserProfile] = None,
    max_chunks: int = 5
):
    """
    Standard RAG retrieval:
    1. Embed the question
    2. Search Qdrant for top-k relevant chunks
    3. Optionally filter by user's hardware level
    4. Construct context and generate answer
    """
    question_embedding = await embed_text(question)

    # Build Qdrant filter based on user profile
    qdrant_filter = None
    if user_profile and user_profile.hardware_tier:
        qdrant_filter = {
            "should": [
                {"key": "hardware_level", "match": {"value": "all"}},
                {"key": "hardware_level", "match": {"value": user_profile.hardware_tier}}
            ]
        }

    # Retrieve top-k chunks
    search_results = await qdrant_client.search(
        collection_name="physical_ai_textbook",
        query_vector=question_embedding,
        query_filter=qdrant_filter,
        limit=max_chunks,
        score_threshold=0.7  # Minimum relevance score
    )

    # Construct context with source tracking
    context_parts = []
    sources = []
    for result in search_results:
        context_parts.append(result.payload["content"])
        sources.append({
            "module": result.payload["module"],
            "chapter": result.payload["chapter"],
            "section": result.payload["section"],
            "file_path": result.payload["file_path"],
            "relevance_score": result.score
        })

    context = "\n\n---\n\n".join(context_parts)

    # Generate answer
    response = await generate_answer(
        question=question,
        context=context,
        system_prompt="You are a Physical AI and Robotics tutor. "
                     "Answer based on the provided context. "
                     "If the context doesn't contain enough information, say so. "
                     "Cite specific modules/chapters in your answer."
    )

    return {
        "answer": response,
        "sources": sources,
        "mode": "full_context"
    }
```

#### Context Window Management
```python
# Token budgets for GPT-4o-mini (128k context, but we limit for cost)
MAX_CONTEXT_TOKENS = 6000  # Retrieved chunks
MAX_QUESTION_TOKENS = 500   # User question
MAX_RESPONSE_TOKENS = 2000  # AI answer
SYSTEM_PROMPT_TOKENS = 200  # System instructions

# Total: ~8700 tokens per request (well under limit, cost-effective)
```

#### Source Attribution Requirements
**ALWAYS return sources in this format:**
```python
{
    "answer": "ROS 2 uses DDS as its middleware...",
    "sources": [
        {
            "module": "Module 1: ROS 2 Fundamentals",
            "chapter": "Chapter 2: Communication Patterns",
            "section": "DDS Middleware",
            "file_path": "docs/module-1/chapter-2-communication.md",
            "relevance_score": 0.92
        }
    ],
    "mode": "full_context",
    "confidence": "high"  # high, medium, low based on relevance scores
}
```

#### Fallback and Error Handling
```python
# When no relevant context found (all scores < 0.7)
if not search_results or all(r.score < 0.7 for r in search_results):
    return {
        "answer": "I don't have enough relevant information in the textbook to answer that question confidently. "
                 "Could you rephrase your question or specify which module/chapter you're asking about?",
        "sources": [],
        "mode": "no_context",
        "confidence": "none"
    }

# When question is ambiguous
if is_ambiguous(question):
    return {
        "answer": "Your question could apply to multiple topics. Did you mean:\n"
                 "1. ROS 2 services (Module 1)\n"
                 "2. Gazebo physics services (Module 2)\n"
                 "3. Isaac Sim services (Module 3)",
        "clarification_needed": True
    }
```

#### Conversation History Management
```python
# Store in Postgres for multi-turn conversations
CREATE TABLE conversation_history (
    id SERIAL PRIMARY KEY,
    conversation_id UUID NOT NULL,
    user_id UUID,
    timestamp TIMESTAMPTZ DEFAULT NOW(),
    role VARCHAR(20),  -- 'user' or 'assistant'
    message TEXT NOT NULL,
    sources JSONB,  -- Source citations for assistant messages
    mode VARCHAR(50)  -- 'selected_text', 'full_context', 'follow_up'
);

# Keep last 5 exchanges in context for follow-up questions
CONVERSATION_CONTEXT_LIMIT = 5
```

### 3. Authentication & Personalization (EXPANDED)

**When working with better-auth integration (`backend/src/services/auth_service.py`):**

#### User Profile Schema
```python
# Database schema
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id),

    -- Hardware profile (from onboarding)
    hardware_tier VARCHAR(20) CHECK (hardware_tier IN ('high_end', 'budget', 'basic')),
    gpu_model VARCHAR(100),  -- e.g., "RTX 4080", "Jetson Orin Nano", "None"
    has_jetson BOOLEAN DEFAULT FALSE,
    has_robot_hardware BOOLEAN DEFAULT FALSE,

    -- Software background (from onboarding)
    software_level VARCHAR(20) CHECK (software_level IN ('beginner', 'intermediate', 'advanced')),
    python_experience VARCHAR(20),  -- 'none', 'basic', 'intermediate', 'expert'
    ros_experience VARCHAR(20),     -- 'none', 'ros1', 'ros2_basic', 'ros2_advanced'
    simulation_experience VARCHAR(20), -- 'none', 'gazebo', 'isaac', 'both'

    -- Preferences
    preferred_language VARCHAR(10) DEFAULT 'en',  -- 'en' or 'ur'
    show_hardware_warnings BOOLEAN DEFAULT TRUE,
    content_verbosity VARCHAR(20) DEFAULT 'balanced',  -- 'concise', 'balanced', 'detailed'

    -- Progress tracking
    completed_modules JSONB DEFAULT '[]',
    current_module VARCHAR(50),
    last_activity TIMESTAMPTZ DEFAULT NOW(),

    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);
```

#### Onboarding Questionnaire Flow
```python
# Step 1: Hardware Assessment
HARDWARE_QUESTIONS = [
    {
        "question": "What GPU do you have access to?",
        "options": [
            {"value": "rtx_4080_plus", "label": "NVIDIA RTX 4080 or better", "tier": "high_end"},
            {"value": "rtx_3060_4070", "label": "NVIDIA RTX 3060-4070", "tier": "high_end"},
            {"value": "jetson", "label": "NVIDIA Jetson (Orin/Xavier)", "tier": "budget"},
            {"value": "no_gpu", "label": "No dedicated GPU", "tier": "basic"}
        ]
    },
    {
        "question": "Do you have access to edge computing hardware?",
        "conditional": True,  # Only show if they selected budget/basic
        "options": [
            {"value": "jetson_orin", "label": "Jetson Orin Nano/NX"},
            {"value": "jetson_xavier", "label": "Jetson Xavier"},
            {"value": "raspberry_pi", "label": "Raspberry Pi"},
            {"value": "none", "label": "None"}
        ]
    },
    {
        "question": "Do you have access to robot hardware?",
        "options": [
            {"value": "humanoid", "label": "Humanoid robot (Unitree G1, etc.)"},
            {"value": "quadruped", "label": "Quadruped (dog robot)"},
            {"value": "arm", "label": "Robotic arm"},
            {"value": "none", "label": "No physical robot"}
        ]
    }
]

# Step 2: Software Background
SOFTWARE_QUESTIONS = [
    {
        "question": "What's your Python experience level?",
        "options": [
            {"value": "expert", "label": "Expert: I write async code, decorators, metaclasses"},
            {"value": "intermediate", "label": "Intermediate: I'm comfortable with classes and libraries"},
            {"value": "basic", "label": "Basic: I can write functions and loops"},
            {"value": "none", "label": "Beginner: Just starting with Python"}
        ]
    },
    {
        "question": "Have you worked with ROS before?",
        "options": [
            {"value": "ros2_advanced", "label": "Yes, ROS 2 (written custom nodes)"},
            {"value": "ros2_basic", "label": "Yes, ROS 2 (followed tutorials)"},
            {"value": "ros1", "label": "Yes, but only ROS 1"},
            {"value": "none", "label": "No ROS experience"}
        ]
    },
    {
        "question": "Simulation experience?",
        "options": [
            {"value": "both", "label": "Both Gazebo and Isaac Sim"},
            {"value": "isaac", "label": "NVIDIA Isaac Sim"},
            {"value": "gazebo", "label": "Gazebo"},
            {"value": "none", "label": "No simulation experience"}
        ]
    }
]
```

#### Personalization Logic

**Content Adaptation Rules:**
```python
async def adapt_content_for_user(
    markdown_content: str,
    user_profile: UserProfile
) -> str:
    """
    Personalize content based on user profile.
    Returns modified markdown with user-specific guidance.
    """
    adaptations = []

    # Hardware-based adaptations
    if user_profile.hardware_tier == "basic":
        # Add cloud alternative sections
        adaptations.append({
            "marker": "<!-- SIMULATION_SECTION -->",
            "replacement": (
                "**💡 Cloud Alternative for Your Hardware**\n"
                "Since you don't have a high-end GPU, consider using:\n"
                "- AWS RoboMaker (pay-per-use)\n"
                "- NVIDIA Omniverse Cloud\n"
                "- Google Colab with limited simulation"
            )
        })
    elif user_profile.hardware_tier == "budget":
        # Emphasize Jetson-compatible approaches
        adaptations.append({
            "marker": "<!-- DEPLOYMENT_NOTE -->",
            "replacement": (
                "**🚀 Jetson Deployment Note**\n"
                "This section is perfect for your Jetson setup. "
                "Focus on inference rather than training."
            )
        })

    # Software level adaptations
    if user_profile.software_level == "beginner":
        # Add more explanatory notes
        adaptations.append({
            "marker": "```python",
            "add_before": (
                "> **Beginner Tip**: The following code might seem complex. "
                "Focus on understanding the overall structure first, "
                "then dive into each function.\n\n"
            )
        })
    elif user_profile.software_level == "advanced":
        # Add advanced alternatives
        adaptations.append({
            "marker": "<!-- ADVANCED_TIP -->",
            "replacement": (
                "**⚡ Advanced**: You might also consider using "
                "ROS 2 lifecycle nodes for better state management."
            )
        })

    # ROS experience adaptations
    if user_profile.ros_experience in ["none", "ros1"]:
        # Add ROS 2 migration notes
        adaptations.append({
            "marker": "<!-- ROS2_NOTE -->",
            "replacement": (
                "**📚 New to ROS 2?** Key differences from ROS 1:\n"
                "- No `roscore` needed (DDS handles discovery)\n"
                "- `colcon build` instead of `catkin_make`\n"
                "- Python 3 only (no Python 2)"
            )
        })

    # Apply adaptations
    adapted_content = markdown_content
    for adaptation in adaptations:
        # Apply marker replacements or insertions
        adapted_content = apply_adaptation(adapted_content, adaptation)

    return adapted_content
```

**Learning Path Recommendations:**
```python
def recommend_next_module(user_profile: UserProfile) -> dict:
    """
    Suggest next module based on completed work and profile.
    """
    completed = set(user_profile.completed_modules)

    # Beginner path (more foundational)
    if user_profile.software_level == "beginner":
        if "module-1" not in completed:
            return {
                "module": "Module 1: ROS 2 Fundamentals",
                "reason": "Start with ROS 2 basics to build a strong foundation",
                "estimated_hours": 15,
                "prerequisites_met": True
            }
        elif "module-2" not in completed and "module-1" in completed:
            return {
                "module": "Module 2: Gazebo Simulation",
                "reason": "Learn simulation before hardware to understand physics",
                "estimated_hours": 12,
                "prerequisites_met": True
            }

    # Advanced path (can skip ahead)
    elif user_profile.software_level == "advanced" and user_profile.ros_experience == "ros2_advanced":
        if "module-3" not in completed:
            return {
                "module": "Module 3: NVIDIA Isaac",
                "reason": "You have the prerequisites to jump to advanced perception",
                "estimated_hours": 20,
                "prerequisites_met": True,
                "optional_review": ["Module 1: ROS 2 Services", "Module 2: Sensor Plugins"]
            }

    # Budget hardware path (emphasize Jetson-friendly content)
    if user_profile.hardware_tier == "budget":
        return {
            "module": "Module 3: Isaac ROS (Jetson Edition)",
            "reason": "Optimized for edge deployment on Jetson hardware",
            "estimated_hours": 18,
            "hardware_requirements": "Jetson Orin Nano or better"
        }

    # Default: sequential progression
    return get_next_sequential_module(completed)
```

### 4. Translation Implementation (EXPANDED)

**When implementing translation (`backend/src/services/translation_service.py`):**

#### Technical Term Glossary
```python
# Preserve these terms in English across all translations
TECHNICAL_TERMS = {
    # Robotics frameworks
    "ROS 2", "ROS 1", "Robot Operating System",
    "Gazebo", "Isaac Sim", "Isaac ROS",
    "Nav2", "MoveIt", "SLAM",

    # Hardware
    "NVIDIA", "Jetson", "Orin", "Xavier",
    "RTX", "GPU", "CPU", "VRAM",
    "LiDAR", "IMU", "RealSense",

    # File formats and protocols
    "URDF", "SDF", "USD", "XML",
    "DDS", "RTPS", "QoS",

    # Programming
    "Python", "C++", "bash", "shell",
    "async", "await", "callback",

    # AI/ML terms
    "VLA", "Vision-Language-Action",
    "transformer", "embedding", "RAG",

    # Physics
    "degrees of freedom", "DOF",
    "kinematics", "dynamics", "PID"
}

# Terms that should be translated but keep English in parentheses
PARTIAL_TRANSLATION_TERMS = {
    "node": "نوڈ (Node)",
    "topic": "ٹاپک (Topic)",
    "service": "سروس (Service)",
    "action": "ایکشن (Action)",
    "publisher": "پبلشر (Publisher)",
    "subscriber": "سبسکرائبر (Subscriber)",
    "package": "پیکیج (Package)",
    "workspace": "ورک سپیس (Workspace)"
}
```

#### Translation Strategy
```python
async def translate_markdown_content(
    content: str,
    source_lang: str = "en",
    target_lang: str = "ur",
    preserve_code: bool = True
) -> dict:
    """
    Translate educational content while preserving technical accuracy.

    Returns:
        {
            "translated_content": str,
            "preserved_terms": List[str],
            "confidence_score": float,
            "code_blocks": List[str]  # Unchanged code blocks
        }
    """

    # Step 1: Extract code blocks and preserve them
    code_blocks = []
    content_without_code = content
    if preserve_code:
        code_pattern = r'```[\s\S]*?```'
        code_blocks = re.findall(code_pattern, content)
        # Replace with placeholders
        for i, block in enumerate(code_blocks):
            content_without_code = content_without_code.replace(
                block, f"<<<CODE_BLOCK_{i}>>>"
            )

    # Step 2: Extract URLs and preserve them
    url_pattern = r'https?://[^\s)]+'
    urls = re.findall(url_pattern, content_without_code)
    for i, url in enumerate(urls):
        content_without_code = content_without_code.replace(
            url, f"<<<URL_{i}>>>"
        )

    # Step 3: Build translation prompt
    system_prompt = f"""You are a technical translator specializing in robotics and AI education.

Translate the following content from {source_lang} to {target_lang}.

CRITICAL RULES:
1. Preserve ALL technical terms from this list IN ENGLISH: {', '.join(TECHNICAL_TERMS)}
2. For common terms, use format: اردو (English) - e.g., {', '.join(PARTIAL_TRANSLATION_TERMS.values())}
3. Keep placeholders like <<<CODE_BLOCK_0>>> EXACTLY as-is
4. Maintain markdown formatting (headers, lists, links)
5. Preserve mathematical notation and equations
6. Keep command-line instructions in English with Urdu explanations

EXAMPLE:
Input: "Create a ROS 2 node to publish sensor data"
Output: "ROS 2 نوڈ بنائیں جو سینسر ڈیٹا پبلش کرے"
"""

    # Step 4: Call OpenAI with streaming for long content
    response = await openai_client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": content_without_code}
        ],
        temperature=0.3,  # Lower temperature for accuracy
        max_tokens=4000
    )

    translated = response.choices[0].message.content

    # Step 5: Restore code blocks
    for i, block in enumerate(code_blocks):
        translated = translated.replace(f"<<<CODE_BLOCK_{i}>>>", block)

    # Step 6: Restore URLs
    for i, url in enumerate(urls):
        translated = translated.replace(f"<<<URL_{i}>>>", url)

    # Step 7: Quality checks
    confidence_score = calculate_translation_confidence(
        original=content,
        translated=translated,
        preserved_terms=TECHNICAL_TERMS
    )

    return {
        "translated_content": translated,
        "preserved_terms": list(TECHNICAL_TERMS),
        "confidence_score": confidence_score,
        "code_blocks": code_blocks,
        "warnings": get_translation_warnings(translated, confidence_score)
    }

def calculate_translation_confidence(
    original: str,
    translated: str,
    preserved_terms: set
) -> float:
    """
    Estimate translation quality.
    Returns: 0.0 to 1.0
    """
    checks = []

    # Check 1: All technical terms preserved
    terms_preserved = all(term in translated for term in preserved_terms if term in original)
    checks.append(1.0 if terms_preserved else 0.5)

    # Check 2: Code blocks unchanged
    orig_code_count = original.count("```")
    trans_code_count = translated.count("```")
    checks.append(1.0 if orig_code_count == trans_code_count else 0.7)

    # Check 3: Markdown structure preserved
    orig_headers = len(re.findall(r'^#+\s', original, re.MULTILINE))
    trans_headers = len(re.findall(r'^#+\s', translated, re.MULTILINE))
    checks.append(1.0 if orig_headers == trans_headers else 0.8)

    # Check 4: Length ratio (translated should be 0.8-1.5x original)
    length_ratio = len(translated) / len(original)
    length_check = 1.0 if 0.8 <= length_ratio <= 1.5 else 0.6
    checks.append(length_check)

    return sum(checks) / len(checks)
```

#### RTL (Right-to-Left) Support
```css
/* frontend/src/css/rtl-support.css */

/* Apply RTL to Urdu content */
.urdu-content {
  direction: rtl;
  text-align: right;
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif;
}

/* Keep code blocks LTR even in RTL context */
.urdu-content pre,
.urdu-content code {
  direction: ltr;
  text-align: left;
  font-family: 'Fira Code', 'Consolas', monospace;
}

/* Keep technical terms in English font */
.urdu-content .technical-term {
  font-family: 'Inter', 'Roboto', sans-serif;
  font-weight: 500;
}

/* Flip navigation elements in RTL */
.urdu-content .pagination-nav {
  direction: rtl;
}

.urdu-content .pagination-nav__link--prev {
  float: right;
}

.urdu-content .pagination-nav__link--next {
  float: left;
}

/* Adjust numbered lists in RTL */
.urdu-content ol {
  padding-right: 2em;
  padding-left: 0;
}

/* Keep inline code snippets readable */
.urdu-content :not(pre) > code {
  direction: ltr;
  unicode-bidi: embed;
}
```

#### Translation Caching Strategy
```python
# Database schema for translation cache
CREATE TABLE translations (
    id SERIAL PRIMARY KEY,
    source_lang VARCHAR(10) NOT NULL,
    target_lang VARCHAR(10) NOT NULL,
    source_content_hash VARCHAR(64) NOT NULL,  -- SHA-256 of source
    source_content TEXT NOT NULL,
    translated_content TEXT NOT NULL,
    confidence_score FLOAT,
    preserved_terms JSONB,
    created_at TIMESTAMPTZ DEFAULT NOW(),
    last_accessed TIMESTAMPTZ DEFAULT NOW(),
    access_count INTEGER DEFAULT 1,

    UNIQUE(source_lang, target_lang, source_content_hash)
);

# Caching logic
async def get_or_create_translation(
    content: str,
    source_lang: str,
    target_lang: str
) -> dict:
    """
    Check cache first, translate if miss.
    """
    content_hash = hashlib.sha256(content.encode()).hexdigest()

    # Try cache first
    cached = await db.execute(
        "SELECT translated_content, confidence_score, preserved_terms "
        "FROM translations "
        "WHERE source_lang = $1 AND target_lang = $2 AND source_content_hash = $3",
        source_lang, target_lang, content_hash
    )

    if cached:
        # Update access stats
        await db.execute(
            "UPDATE translations SET last_accessed = NOW(), access_count = access_count + 1 "
            "WHERE source_content_hash = $1",
            content_hash
        )
        return {
            "translated_content": cached["translated_content"],
            "confidence_score": cached["confidence_score"],
            "cached": True
        }

    # Cache miss: translate and store
    result = await translate_markdown_content(content, source_lang, target_lang)

    await db.execute(
        "INSERT INTO translations "
        "(source_lang, target_lang, source_content_hash, source_content, "
        " translated_content, confidence_score, preserved_terms) "
        "VALUES ($1, $2, $3, $4, $5, $6, $7)",
        source_lang, target_lang, content_hash, content,
        result["translated_content"], result["confidence_score"],
        json.dumps(result["preserved_terms"])
    )

    result["cached"] = False
    return result
```

---

## Spec-Driven Development (SDD) Workflow

### Core Guarantees (Product Promise)

- Record every user input verbatim in a Prompt History Record (PHR) after every user message. Do not truncate; preserve full multiline input.
- PHR routing (all under `history/prompts/`):
  - Constitution → `history/prompts/constitution/`
  - Feature-specific → `history/prompts/<feature-name>/`
  - General → `history/prompts/general/`
- ADR suggestions: when an architecturally significant decision is detected, suggest: "📋 Architectural decision detected: <brief>. Document? Run `/sp.adr <title>`." Never auto‑create ADRs; require user consent.

### Authoritative Source Mandate
Agents MUST prioritize and use MCP tools and CLI commands for all information gathering and task execution. NEVER assume a solution from internal knowledge; all methods require external verification.

### Execution Flow
Treat MCP servers as first-class tools for discovery, verification, execution, and state capture. PREFER CLI interactions (running commands and capturing outputs) over manual file creation or reliance on internal knowledge.

### Knowledge Capture (PHR) for Every User Input

After completing requests, you **MUST** create a PHR (Prompt History Record).

**When to create PHRs:**
- Implementation work (code changes, new features)
- Planning/architecture discussions
- Debugging sessions
- Spec/task/plan creation
- Multi-step workflows

**PHR Creation Process:**

1) **Detect stage**
   - One of: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) **Generate title**
   - 3–7 words; create a slug for the filename.

3) **Resolve route** (all under history/prompts/)
   - `constitution` → `history/prompts/constitution/`
   - Feature stages (spec, plan, tasks, red, green, refactor, explainer, misc) → `history/prompts/<feature-name>/` (requires feature context)
   - `general` → `history/prompts/general/`

4) **Prefer agent‑native flow** (no shell)
   - Read the PHR template from one of:
     - `.specify/templates/phr-template.prompt.md`
     - `templates/phr-template.prompt.md`
   - Allocate an ID (increment; on collision, increment again).
   - Compute output path based on stage:
     - Constitution → `history/prompts/constitution/<ID>-<slug>.constitution.prompt.md`
     - Feature → `history/prompts/<feature-name>/<ID>-<slug>.<stage>.prompt.md`
     - General → `history/prompts/general/<ID>-<slug>.general.prompt.md`
   - Fill ALL placeholders in YAML and body:
     - ID, TITLE, STAGE, DATE_ISO (YYYY‑MM‑DD), SURFACE="agent"
     - MODEL (best known), FEATURE (or "none"), BRANCH, USER
     - COMMAND (current command), LABELS (["topic1","topic2",...])
     - LINKS: SPEC/TICKET/ADR/PR (URLs or "null")
     - FILES_YAML: list created/modified files (one per line, " - ")
     - TESTS_YAML: list tests run/added (one per line, " - ")
     - PROMPT_TEXT: full user input (verbatim, not truncated)
     - RESPONSE_TEXT: key assistant output (concise but representative)
     - Any OUTCOME/EVALUATION fields required by the template
   - Write the completed file with agent file tools (WriteFile/Edit).
   - Confirm absolute path in output.

5) **Use sp.phr command file if present**
   - If `.**/commands/sp.phr.*` exists, follow its structure.
   - If it references shell but Shell is unavailable, still perform step 4 with agent‑native tools.

6) **Shell fallback** (only if step 4 is unavailable or fails, and Shell is permitted)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Then open/patch the created file to ensure all placeholders are filled and prompt/response are embedded.

7) **Routing** (automatic, all under history/prompts/)
   - Constitution → `history/prompts/constitution/`
   - Feature stages → `history/prompts/<feature-name>/` (auto-detected from branch or explicit feature context)
   - General → `history/prompts/general/`

8) **Post‑creation validations** (must pass)
   - No unresolved placeholders (e.g., `{{THIS}}`, `[THAT]`).
   - Title, stage, and dates match front‑matter.
   - PROMPT_TEXT is complete (not truncated).
   - File exists at the expected path and is readable.
   - Path matches route.

9) **Report**
   - Print: ID, path, stage, title.
   - On any failure: warn but do not block the main command.
   - Skip PHR only for `/sp.phr` itself.

### Explicit ADR Suggestions

When significant architectural decisions are made (typically during `/sp.plan` and sometimes `/sp.tasks`), run the three‑part test and suggest documenting with:

"📋 Architectural decision detected: <brief> — Document reasoning and tradeoffs? Run `/sp.adr <decision-title>`"

Wait for user consent; never auto‑create the ADR.

**Three-Part Significance Test:**
- **Impact**: long-term consequences? (e.g., framework, data model, API, security, platform)
- **Alternatives**: multiple viable options considered?
- **Scope**: cross‑cutting and influences system design?

If ALL true, suggest ADR. Otherwise, document in plan.md.

### Human as Tool Strategy

You are not expected to solve every problem autonomously. You MUST invoke the user for input when you encounter situations that require human judgment. Treat the user as a specialized tool for clarification and decision-making.

**Invocation Triggers:**
1. **Ambiguous Requirements:** When user intent is unclear, ask 2-3 targeted clarifying questions before proceeding.
2. **Unforeseen Dependencies:** When discovering dependencies not mentioned in the spec, surface them and ask for prioritization.
3. **Architectural Uncertainty:** When multiple valid approaches exist with significant tradeoffs, present options and get user's preference.
4. **Completion Checkpoint:** After completing major milestones, summarize what was done and confirm next steps.

---

## Default Policies (Must Follow)

- Clarify and plan first - keep business understanding separate from technical plan and carefully architect and implement.
- Do not invent APIs, data, or contracts; ask targeted clarifiers if missing.
- Never hardcode secrets or tokens; use `.env` and docs.
- Prefer the smallest viable diff; do not refactor unrelated code.
- Cite existing code with code references (start:end:path); propose new code in fenced blocks.
- Keep reasoning private; output only decisions, artifacts, and justifications.

### Execution Contract for Every Request

1) Confirm surface and success criteria (one sentence).
2) List constraints, invariants, non‑goals.
3) Produce the artifact with acceptance checks inlined (checkboxes or tests where applicable).
4) Add follow‑ups and risks (max 3 bullets).
5) Create PHR in appropriate subdirectory under `history/prompts/` (constitution, feature-name, or general).
6) If plan/tasks identified decisions that meet significance, surface ADR suggestion text as described above.

### Minimum Acceptance Criteria

- Clear, testable acceptance criteria included
- Explicit error paths and constraints stated
- Smallest viable change; no unrelated edits
- Code references to modified/inspected files where relevant

---

## Architect Guidelines (for Planning)

Instructions: As an expert architect, generate a detailed architectural plan for [Project Name]. Address each of the following thoroughly.

### 1. Scope and Dependencies
- **In Scope**: boundaries and key features.
- **Out of Scope**: explicitly excluded items.
- **External Dependencies**: systems/services/teams and ownership.

### 2. Key Decisions and Rationale
- **Options Considered**, Trade-offs, Rationale.
- **Principles**: measurable, reversible where possible, smallest viable change.

### 3. Interfaces and API Contracts
- **Public APIs**: Inputs, Outputs, Errors.
- **Versioning Strategy**.
- **Idempotency, Timeouts, Retries**.
- **Error Taxonomy** with status codes.

### 4. Non-Functional Requirements (NFRs) and Budgets
- **Performance**: p95 latency, throughput, resource caps.
- **Reliability**: SLOs, error budgets, degradation strategy.
- **Security**: AuthN/AuthZ, data handling, secrets, auditing.
- **Cost**: unit economics.

### 5. Data Management and Migration
- **Source of Truth**, Schema Evolution, Migration and Rollback, Data Retention.

### 6. Operational Readiness
- **Observability**: logs, metrics, traces.
- **Alerting**: thresholds and on-call owners.
- **Runbooks** for common tasks.
- **Deployment and Rollback** strategies.
- **Feature Flags** and compatibility.

### 7. Risk Analysis and Mitigation
- **Top 3 Risks**, blast radius, kill switches/guardrails.

### 8. Evaluation and Validation
- **Definition of Done** (tests, scans).
- **Output Validation** for format/requirements/safety.

### 9. Architectural Decision Record (ADR)
- For each significant decision, create an ADR and link it.

### Architecture Decision Records (ADR) - Intelligent Suggestion

After design/architecture work, test for ADR significance:

- **Impact**: long-term consequences? (e.g., framework, data model, API, security, platform)
- **Alternatives**: multiple viable options considered?
- **Scope**: cross‑cutting and influences system design?

If ALL true, suggest:
```
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`
```

Wait for consent; never auto-create ADRs. Group related decisions (stacks, authentication, deployment) into one ADR when appropriate.

---

## Basic Project Structure

```
.
├── .specify/
│   ├── memory/
│   │   └── constitution.md      # Project principles
│   ├── templates/               # PHR, ADR templates
│   └── scripts/                 # Automation scripts
├── specs/
│   └── <feature>/
│       ├── spec.md              # Feature requirements
│       ├── plan.md              # Architecture decisions
│       └── tasks.md             # Testable tasks with cases
├── history/
│   ├── prompts/                 # Prompt History Records
│   │   ├── constitution/
│   │   ├── <feature-name>/
│   │   └── general/
│   └── adr/                     # Architecture Decision Records
├── backend/
│   ├── src/
│   │   ├── api/                 # FastAPI routes
│   │   ├── services/            # Business logic
│   │   ├── models/              # Data models
│   │   └── db/                  # Database
│   └── tests/
├── frontend/
│   ├── docs/                    # Educational content
│   ├── src/
│   │   ├── components/          # React components
│   │   └── css/                 # Styles
│   └── static/                  # Assets
├── CLAUDE.md                    # This file
└── README.md
```

---

## Code Standards

### Python (Backend)
- **Style**: Follow PEP 8, use `black` formatter, `flake8` linter
- **Type Hints**: All function signatures must have type annotations
- **Error Handling**: Use custom exceptions, never bare `except:`
- **Async/Await**: Use async for I/O operations (DB, API calls, embeddings)
- **Testing**: Minimum 80% code coverage with pytest
- **Docstrings**: Use Google-style docstrings for all public functions

### TypeScript/React (Frontend)
- **Style**: Follow Airbnb style guide, use ESLint + Prettier
- **Components**: Functional components with hooks (no class components)
- **Props**: Define PropTypes or use TypeScript interfaces
- **State Management**: React Context for global state (auth, theme, language)
- **Accessibility**: ARIA labels, keyboard navigation, semantic HTML
- **Performance**: Memoization for expensive computations, lazy loading for routes

### Markdown (Educational Content)
- **Headings**: Use hierarchical structure (H1 → H2 → H3, no skipping)
- **Code Blocks**: Always specify language (```python, ```bash, ```xml)
- **Links**: Use relative paths for internal links, absolute for external
- **Images**: Store in `static/img/`, use descriptive alt text, optimize size
- **Lists**: Use ordered lists for sequential steps, unordered for non-sequential items

---

## Testing Requirements

### Backend Tests (`backend/tests/`)
- **Unit Tests**: Individual functions (e.g., `test_embedding_generation()`)
- **Integration Tests**: API endpoints with mocked external services (OpenAI, Qdrant)
- **RAG Tests**: Validate retrieval accuracy, context relevance, source attribution
- **Auth Tests**: Signup, signin, session management, profile updates
- **Translation Tests**: Technical term preservation, RTL handling, cache behavior

### Frontend Tests (`frontend/src/`)
- **Component Tests**: React Testing Library for UI components
- **Integration Tests**: Test ChatBot with mocked backend
- **Accessibility Tests**: ARIA compliance, keyboard navigation
- **Build Tests**: `npm run build` must succeed without errors
- **E2E Tests** (Optional): Playwright for critical user flows

### Quality Gates (Must Pass Before Deployment)
- [ ] All backend tests pass (`pytest`)
- [ ] All frontend tests pass (`npm test`)
- [ ] No TypeScript errors (`tsc --noEmit`)
- [ ] Linting passes (`flake8`, `eslint`)
- [ ] Test coverage ≥ 80% for backend critical paths
- [ ] Security scan passes (no high/critical vulnerabilities)
- [ ] Build succeeds (`npm run build`)

---

## Performance & Optimization

### For GitHub Pages Deployment
- **Bundle Size**: Keep main bundle under 500KB (use code splitting with React.lazy)
- **Image Optimization**: Compress images, use WebP format, lazy load below fold
- **Search Index**: Limit Docusaurus search plugin to essential pages
- **API Calls**: Debounce chatbot queries (500ms delay), implement request cancellation
- **Caching**: Use browser cache for static assets, service worker for offline support

### For RAG Performance
- **Vector Search**: Limit Qdrant results to top 5 most relevant chunks
- **Streaming Responses**: Use SSE (Server-Sent Events) for long AI responses
- **Rate Limiting**: Implement per-user rate limits (10 requests/minute)
- **Connection Pooling**: Reuse database connections, limit concurrent queries
- **CDN**: Serve static assets from CDN (GitHub Pages CDN built-in)

---

## Deployment Checklist

Before deploying to GitHub Pages:

### Code Quality
- [ ] All tests pass (backend + frontend)
- [ ] No linting errors
- [ ] No TypeScript errors
- [ ] Code coverage meets minimum threshold

### Configuration
- [ ] Environment variables documented in `.env.example`
- [ ] CORS configured for frontend domain (https://<username>.github.io)
- [ ] API rate limits implemented
- [ ] Database migrations applied
- [ ] Qdrant collections created and indexed

### Content
- [ ] All educational content proofread
- [ ] Code examples tested and working
- [ ] Images optimized and have alt text
- [ ] Internal links validated (no 404s)
- [ ] Translation cache warmed for common content

### Security
- [ ] No secrets in source code
- [ ] API authentication working
- [ ] HTTPS enforced
- [ ] Input validation on all endpoints
- [ ] SQL injection prevention (parameterized queries)

### User Experience
- [ ] Mobile responsive design
- [ ] Fast page load (< 3s on 3G)
- [ ] Error pages customized (404.html)
- [ ] Loading states for async operations
- [ ] RTL support tested for Urdu

### Documentation
- [ ] README.md updated with setup instructions
- [ ] API documentation current
- [ ] License file present
- [ ] Contribution guidelines (if open source)

---

## Quick Reference Commands

### Development

```bash
# Backend
cd backend
python -m venv venv
venv\Scripts\activate  # Windows
source venv/bin/activate  # Linux/Mac
pip install -r requirements.txt
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000

# Frontend
cd frontend
npm install
npm start  # Development server
npm run build  # Production build
npm test  # Run tests

# Testing
cd backend && pytest --cov=src tests/
cd frontend && npm test -- --coverage

# Linting
cd backend && flake8 src/ && black src/
cd frontend && npm run lint && npm run format
```

### Slash Commands (Spec-Kit Plus)

- `/sp.specify` - Create/update feature specification
- `/sp.plan` - Generate implementation plan
- `/sp.tasks` - Create task breakdown
- `/sp.implement` - Execute tasks from tasks.md
- `/sp.phr` - Create Prompt History Record
- `/sp.adr` - Create Architecture Decision Record
- `/sp.constitution` - Create/update project constitution
- `/sp.clarify` - Identify underspecified areas in spec

---

## Common Patterns and Anti-Patterns

### ✅ Good Patterns

**RAG Query with Source Attribution:**
```python
# Good: Always return sources
{
    "answer": "ROS 2 uses DDS...",
    "sources": [{"module": "Module 1", "chapter": "Chapter 2"}],
    "confidence": "high"
}
```

**Hardware-Aware Content:**
```markdown
### For RTX 4080+ Users
Run Isaac Sim locally for photorealistic rendering.

### For Budget Hardware Users
Use AWS RoboMaker or Gazebo Classic.
```

**Translation with Term Preservation:**
```python
# Good: Technical terms stay in English
"ROS 2 نوڈ بنائیں"  # "Create a ROS 2 node"
```

### ❌ Anti-Patterns

**RAG without Sources:**
```python
# Bad: No way to verify answer
{"answer": "ROS 2 uses DDS..."}
```

**Generic Content for All Users:**
```markdown
# Bad: Assumes everyone has high-end hardware
First, install Isaac Sim (requires RTX GPU)...
```

**Translating Technical Terms:**
```python
# Bad: Breaks technical meaning
"روبوٹ آپریٹنگ سسٹم 2"  # Don't translate "ROS 2"
```

**Inventing APIs:**
```python
# Bad: No external verification
def get_user_profile(user_id):
    # Assuming this endpoint exists...
```

---

## Final Reminders

1. **Always create PHRs** after completing user requests
2. **Suggest ADRs** for architecturally significant decisions (don't auto-create)
3. **Ask clarifying questions** when requirements are ambiguous
4. **Prioritize educational content accuracy** - this is a textbook, not a blog
5. **Test on actual hardware tiers** - what works on RTX 4080 may not work on Jetson
6. **Preserve technical terms** in translations - accuracy over fluency
7. **Cite sources** in RAG responses - no hallucination
8. **Use MCP tools** for verification - don't assume from internal knowledge

---

**See `.specify/memory/constitution.md` for additional code quality, testing, performance, security, and architecture principles.**
