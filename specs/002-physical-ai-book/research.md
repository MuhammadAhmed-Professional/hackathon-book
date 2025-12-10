# Research: Physical AI & Humanoid Robotics Textbook

**Date**: 2025-11-27
**Feature**: 002-physical-ai-book
**Purpose**: Research findings to inform textbook content creation, Neon Postgres integration, Better-auth setup, and content structure

---

## 1. Physical AI Course Content

### Key Concepts by Module

#### Module 1: ROS 2 - The Robotic Nervous System (Weeks 3-5)

**Core Concepts**:
- **ROS 2 Architecture**: Middleware for robot control, DDS (Data Distribution Service) foundation, real-time communication
- **Nodes**: Independent processes that perform computation, communicate via topics and services
- **Topics**: Asynchronous publish/subscribe messaging for continuous data streams (sensor data, motor commands)
- **Services**: Synchronous request/response patterns for discrete operations (calibration, configuration)
- **Actions**: Long-running tasks with feedback and cancellation (navigation, manipulation)
- **rclpy**: Python client library for ROS 2, creating nodes, publishers, subscribers, service clients/servers
- **URDF**: Unified Robot Description Format for humanoid robot modeling (links, joints, kinematic chains)
- **Launch Files**: XML/Python scripts to start multiple nodes with parameters

**Key Learning Objectives**:
- Understand ROS 2 as middleware bridging AI algorithms to physical hardware
- Create ROS 2 nodes in Python using rclpy
- Implement publish/subscribe communication for sensor data
- Define humanoid robot structure using URDF
- Launch complex robotic systems with parameter management

**Chapter Topics**:
1. Introduction to ROS 2 and Physical AI
2. ROS 2 Architecture and Core Concepts
3. Nodes, Topics, and Services
4. Building ROS 2 Packages with Python (rclpy)
5. URDF for Humanoid Robots
6. Launch Files and Parameter Management

---

#### Module 2: Gazebo & Unity - The Digital Twin (Weeks 6-7)

**Core Concepts**:
- **Physics Simulation**: Gravity, collisions, friction, rigid body dynamics in Gazebo
- **URDF/SDF**: Robot description formats (URDF for ROS integration, SDF for Gazebo-specific features)
- **Sensor Simulation**: LiDAR (point clouds), depth cameras (RGB-D), IMUs (orientation, acceleration)
- **Gazebo Integration**: Spawning robots, simulating environments, plugin architecture
- **Unity for Rendering**: High-fidelity visualization, human-robot interaction scenarios, asset libraries

**Key Learning Objectives**:
- Simulate humanoid robots in realistic physics environments
- Understand sensor simulation for perception testing
- Use URDF/SDF to define robot models with visual and collision properties
- Integrate Gazebo with ROS 2 for testing before hardware deployment
- Leverage Unity for photorealistic rendering and visualization

**Chapter Topics**:
1. Introduction to Robot Simulation
2. Gazebo Environment Setup
3. URDF and SDF Robot Description Formats
4. Physics Simulation (Gravity, Collisions, Friction)
5. Sensor Simulation (LiDAR, Depth Cameras, IMUs)
6. Unity for High-Fidelity Rendering

---

#### Module 3: NVIDIA Isaac - The AI-Robot Brain (Weeks 8-10)

**Core Concepts**:
- **Isaac Sim**: NVIDIA Omniverse-based photorealistic simulation, synthetic data generation for training perception models
- **Isaac ROS**: Hardware-accelerated perception packages (VSLAM, object detection, pose estimation)
- **VSLAM**: Visual Simultaneous Localization and Mapping for robot navigation using camera data
- **Nav2**: Navigation stack for path planning, obstacle avoidance, and goal-reaching in 2D/3D environments
- **Sim-to-Real**: Training policies in simulation and transferring to physical robots
- **USD**: Universal Scene Description format for 3D assets in Isaac Sim

**Key Learning Objectives**:
- Use Isaac Sim for photorealistic robot simulation and data generation
- Implement VSLAM for humanoid robot localization and mapping
- Apply Nav2 for autonomous navigation and path planning
- Understand sim-to-real transfer techniques for deploying learned behaviors
- Leverage GPU acceleration for real-time perception

**Chapter Topics**:
1. Introduction to NVIDIA Isaac Platform
2. Isaac Sim: Photorealistic Simulation
3. Synthetic Data Generation for AI Training
4. Isaac ROS: Hardware-Accelerated Perception
5. VSLAM and Navigation with Isaac ROS
6. Nav2 Path Planning for Bipedal Humanoids
7. Sim-to-Real Transfer Techniques

---

#### Module 4: Vision-Language-Action (VLA) - Convergence of LLMs and Robotics (Week 13)

**Core Concepts**:
- **Voice-to-Action**: Using OpenAI Whisper for speech recognition, translating voice commands to robot actions
- **Cognitive Planning**: Using LLMs (GPT-4) to decompose natural language instructions into ROS 2 action sequences
- **Multimodal AI**: Combining vision (cameras), language (LLMs), and action (motor control) for intelligent behavior
- **Capstone Project**: Autonomous humanoid receives voice command, plans path, navigates obstacles, identifies object, manipulates it

**Key Learning Objectives**:
- Integrate speech recognition (Whisper) with robotic control systems
- Use LLMs for high-level task planning and reasoning
- Implement vision-language-action pipelines for natural human-robot interaction
- Design and execute a capstone project demonstrating autonomous humanoid capabilities

**Chapter Topics**:
1. Introduction to Vision-Language-Action (VLA)
2. Voice-to-Action with OpenAI Whisper
3. Cognitive Planning with LLMs
4. Multimodal AI for Robotics
5. Capstone Project: The Autonomous Humanoid

---

### Learning Objectives (13-Week Course)

**Weeks 1-2: Introduction**
- Understand Physical AI and embodied intelligence
- Distinguish digital AI from AI systems in physical environments
- Overview of humanoid robotics landscape and applications

**Weeks 3-5: ROS 2 Fundamentals**
- Master ROS 2 architecture and communication patterns
- Build ROS 2 packages with Python (rclpy)
- Model humanoid robots with URDF

**Weeks 6-7: Simulation with Gazebo & Unity**
- Simulate robots in physics-based environments
- Test sensor systems (LiDAR, depth cameras, IMUs)
- Visualize robots with Unity rendering

**Weeks 8-10: NVIDIA Isaac Platform**
- Use Isaac Sim for photorealistic simulation
- Implement VSLAM and navigation with Isaac ROS
- Apply Nav2 for path planning

**Weeks 11-12: Humanoid Robot Development**
- Understand humanoid kinematics and dynamics
- Implement bipedal locomotion and balance control
- Design natural human-robot interaction

**Week 13: Conversational Robotics**
- Integrate GPT models for conversational AI
- Implement voice-to-action pipelines
- Complete capstone project: autonomous humanoid

---

### Chapter Structure Recommendations

**Target Chapter Length**: 800-1,200 words per chapter
- **Rationale**: Comprehensive coverage without overwhelming students. Enough depth for understanding concepts, short enough to maintain engagement.

**Chapter Format**:
1. **Introduction** (100-150 words): Overview of chapter topic and learning objectives
2. **Core Concepts** (400-600 words): Explain key ideas with examples and diagrams
3. **Code Examples** (200-300 words): Practical implementations with syntax highlighting
4. **Hardware/Tools** (100-200 words): Required tools, software, or hardware for topic
5. **Summary** (100-150 words): Recap key takeaways and next steps

**Code Example Format**:
- Use fenced code blocks with language specifiers (```python, ```bash, ```xml)
- Include comments explaining each section
- Provide complete, runnable examples when possible
- Link to external repositories for longer examples

**Diagram Types**:
- Architecture diagrams (ROS 2 nodes and topics)
- Flowcharts (navigation algorithms, planning pipelines)
- Hardware diagrams (sensor placement, robot kinematics)
- Screenshots (Gazebo simulations, Isaac Sim environments)

---

## 2. Neon Serverless Postgres Integration

### Connection Patterns

**psycopg2 vs psycopg3**:
- **Decision**: Use **psycopg2-binary** for this project
- **Rationale**:
  - Mature, widely adopted (most Stack Overflow answers, tutorials)
  - Excellent FastAPI integration with existing patterns
  - Neon officially supports psycopg2
  - psycopg3 is newer (async-first) but has breaking API changes and fewer community resources
- **Installation**: `pip install psycopg2-binary` (binary version avoids compilation issues)

**Async Database Operations with FastAPI**:
- **Pattern**: Use `asyncpg` for async operations OR use `psycopg2` with thread pool executor
- **Decision for hackathon**: Use **psycopg2 with synchronous operations** (simpler, fewer edge cases)
- **Rationale**: FastAPI can handle sync database calls efficiently with thread pools. Async adds complexity (connection pool management, transaction handling) that's not critical for hackathon scale (10-50 concurrent users).

**Connection String Format** (Neon Postgres):
```
postgresql://user:password@host.region.neon.tech/dbname?sslmode=require
```
- SSL is **required** for Neon connections
- Parse using `urllib.parse` or pass directly to psycopg2.connect()

---

### Schema Design

**Users Table**:
```sql
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    software_background JSONB,
    hardware_background JSONB,
    created_at TIMESTAMP DEFAULT NOW(),
    last_login TIMESTAMP
);

CREATE INDEX idx_users_email ON users(email);
```

**Conversations Table**:
```sql
CREATE TABLE conversations (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE SET NULL,
    session_id VARCHAR(255),
    question TEXT NOT NULL,
    answer TEXT NOT NULL,
    citations JSONB,
    question_type VARCHAR(50) CHECK (question_type IN ('rag', 'selected_text')),
    timestamp TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_conversations_user_id ON conversations(user_id);
CREATE INDEX idx_conversations_session_id ON conversations(session_id);
CREATE INDEX idx_conversations_timestamp ON conversations(timestamp DESC);
```

**Design Decisions**:
- `user_id` is **nullable** (allows anonymous conversations with session_id)
- `ON DELETE SET NULL` preserves conversations if user account deleted
- JSONB columns for flexible schema (`software_background`, `hardware_background`, `citations`)
- Indexes on frequently queried columns (email, user_id, session_id, timestamp)

---

### JSONB Usage

**When to Use JSONB**:
- Semi-structured data with variable fields (user background questionnaires can evolve)
- Data that doesn't require strict validation at database level
- Nested structures (citations array with module, chapter, chunk_id, relevance_score)
- Agile development where schema changes frequently

**Example JSONB Data**:

```json
// software_background
{
  "programming_languages": ["Python", "C++", "JavaScript"],
  "robotics_experience": "intermediate",
  "ai_ml_level": "advanced",
  "prior_courses": ["CS101", "Robotics 101"]
}

// hardware_background
{
  "rtx_gpu_access": true,
  "rtx_gpu_model": "RTX 4070 Ti",
  "jetson_kit": "Orin Nano 8GB",
  "robot_hardware": "none"
}

// citations
[
  {
    "module": "Module 1: ROS 2",
    "chapter": "Nodes and Topics",
    "chunk_id": "chunk_42",
    "relevance_score": 0.89
  },
  {
    "module": "Module 3: NVIDIA Isaac",
    "chapter": "Isaac ROS",
    "chunk_id": "chunk_156",
    "relevance_score": 0.75
  }
]
```

**Querying JSONB**:
```sql
-- Find users with RTX GPU access
SELECT * FROM users WHERE software_background->>'rtx_gpu_access' = 'true';

-- Find conversations with citations from Module 1
SELECT * FROM conversations WHERE citations @> '[{"module": "Module 1: ROS 2"}]';
```

---

### Connection Pooling Best Practices

**Pattern**: Use a connection pool to reuse database connections across requests

```python
import psycopg2
from psycopg2 import pool

# Initialize connection pool (call once at app startup)
connection_pool = psycopg2.pool.SimpleConnectionPool(
    minconn=1,
    maxconn=10,
    dsn=os.getenv("NEON_DATABASE_URL")
)

# Get connection from pool
conn = connection_pool.getconn()
try:
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM users WHERE email = %s", (email,))
    result = cursor.fetchone()
finally:
    cursor.close()
    connection_pool.putconn(conn)  # Return connection to pool
```

**Best Practices**:
- Initialize pool once at FastAPI app startup (lifespan event)
- Set `maxconn` based on Neon free tier limits (recommended: 10-20)
- Always return connections to pool in `finally` block
- Use parameterized queries to prevent SQL injection (`%s` placeholders)

---

## 3. Better-auth Integration

### Setup with React/Docusaurus

**Installation**:
```bash
npm install better-auth
```

**Better-auth Configuration** (React side):
```typescript
// src/lib/auth.ts
import { createAuthClient } from 'better-auth/client';

export const authClient = createAuthClient({
  baseURL: 'http://localhost:8000',  // FastAPI backend
  credentials: 'include',  // Send cookies with requests
});
```

**FastAPI Backend Integration**:
- Better-auth is primarily a frontend library, but backend needs to:
  1. Provide `/auth/signup` and `/auth/signin` endpoints
  2. Generate JWT tokens on successful authentication
  3. Validate JWT tokens on protected endpoints

**Simplified Approach for Hackathon**:
- Use Better-auth for **UI components only** (signup/signin forms)
- Implement **custom JWT backend** with FastAPI (simpler than full Better-auth server integration)
- Use `python-jose` for JWT generation/validation
- Use `passlib` for password hashing

---

### JWT Token Management

**JWT Generation** (Backend - FastAPI):
```python
from jose import jwt
from datetime import datetime, timedelta

SECRET_KEY = os.getenv("BETTER_AUTH_SECRET")  # From environment
ALGORITHM = "HS256"

def create_access_token(user_id: int, email: str) -> str:
    payload = {
        "user_id": user_id,
        "email": email,
        "exp": datetime.utcnow() + timedelta(hours=24)
    }
    return jwt.encode(payload, SECRET_KEY, algorithm=ALGORITHM)
```

**JWT Validation** (Backend - FastAPI dependency):
```python
from fastapi import HTTPException, Depends
from fastapi.security import HTTPBearer

security = HTTPBearer()

def get_current_user(token: str = Depends(security)):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id = payload.get("user_id")
        if user_id is None:
            raise HTTPException(status_code=401, detail="Invalid token")
        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token expired")
    except jwt.JWTError:
        raise HTTPException(status_code=401, detail="Invalid token")
```

**Token Storage** (Frontend):
- **Decision**: Use **httpOnly cookies** for production, **localStorage** for development
- **Rationale**: httpOnly cookies prevent XSS attacks (JavaScript can't access token), but require backend CORS configuration with `credentials: 'include'`
- **Development fallback**: localStorage is easier to debug and doesn't require cookie configuration

---

### Authentication Flows

**Signup Flow**:
1. User submits email, password, software_background, hardware_background via `SignupForm.tsx`
2. Frontend sends POST to `/auth/signup` with form data
3. Backend validates email uniqueness, hashes password with bcrypt
4. Backend inserts user into `users` table
5. Backend generates JWT token with user_id and email
6. Backend returns `{user_id, email, token}` to frontend
7. Frontend stores token in localStorage/cookie
8. Frontend redirects to textbook homepage with authenticated state

**Signin Flow**:
1. User submits email and password via `SigninForm.tsx`
2. Frontend sends POST to `/auth/signin`
3. Backend queries `users` table for email
4. Backend verifies password hash with bcrypt.checkpw()
5. If valid, backend generates JWT token
6. Backend updates `last_login` timestamp
7. Backend returns `{user_id, email, token}` to frontend
8. Frontend stores token and redirects to textbook

**Protected Endpoint Flow** (e.g., `/ask`):
1. Frontend includes JWT in `Authorization: Bearer <token>` header
2. Backend validates token with `get_current_user` dependency
3. If valid, extract user_id from token payload
4. Save conversation to Neon Postgres with user_id
5. Return answer to frontend

---

### Password Hashing

**Library**: Use `passlib` with bcrypt algorithm

```python
from passlib.context import CryptContext

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# Hash password on signup
hashed_password = pwd_context.hash(plain_password)

# Verify password on signin
is_valid = pwd_context.verify(plain_password, hashed_password)
```

**Security Best Practices**:
- Never store plain-text passwords
- Use bcrypt (slow hashing algorithm resistant to brute-force)
- Salt is automatically included in bcrypt hashes
- Validate password strength on frontend (min 8 characters, mix of letters/numbers)

---

## 4. Textbook Content Structure

### Docusaurus Best Practices

**Nested Sidebar Structure**:
```javascript
// sidebars.js
module.exports = {
  tutorialSidebar: [
    'intro',  // Introduction page
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'module1/index',
        'module1/ros2-architecture',
        'module1/nodes-topics-services',
        'module1/python-integration',
        'module1/urdf-for-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo & Unity',
      items: [
        'module2/index',
        'module2/gazebo-simulation',
        'module2/urdf-sdf-formats',
        'module2/physics-simulation',
        'module2/unity-rendering',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      items: [
        'module3/index',
        'module3/isaac-sim',
        'module3/isaac-ros',
        'module3/vslam-navigation',
        'module3/nav2-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA',
      items: [
        'module4/index',
        'module4/voice-to-action',
        'module4/cognitive-planning',
        'module4/capstone-project',
      ],
    },
    'hardware',
    'weekly-breakdown',
  ],
};
```

**Directory Structure**:
```
frontend/docs/
├── intro.md
├── module1/
│   ├── index.md
│   ├── ros2-architecture.md
│   ├── nodes-topics-services.md
│   ├── python-integration.md
│   └── urdf-for-humanoids.md
├── module2/
│   ├── index.md
│   ├── gazebo-simulation.md
│   ├── urdf-sdf-formats.md
│   ├── physics-simulation.md
│   └── unity-rendering.md
├── module3/
│   ├── index.md
│   ├── isaac-sim.md
│   ├── isaac-ros.md
│   ├── vslam-navigation.md
│   └── nav2-planning.md
├── module4/
│   ├── index.md
│   ├── voice-to-action.md
│   ├── cognitive-planning.md
│   └── capstone-project.md
├── hardware.md
└── weekly-breakdown.md
```

---

### Educational Design

**Module Organization Principles**:
- **Scaffolding**: Start with foundational concepts (ROS 2) before advanced topics (Isaac, VLA)
- **Modularity**: Each module is self-contained but builds on previous modules
- **Progression**: Move from theory (what is ROS 2?) to practice (build a ROS 2 package)
- **Real-world context**: Tie concepts to humanoid robotics applications

**Chapter Organization**:
- Each module has an **index.md** introducing the module's goals and chapters
- Chapters within a module follow logical progression (architecture → implementation → integration)
- Cross-reference related chapters (e.g., URDF in Module 1 connects to Gazebo in Module 2)

---

### Navigation Strategy

**Sidebar Navigation**:
- Collapsible categories for each module (4 modules)
- Index page for each module appears first in category
- Linear progression through chapters (1 → 2 → 3 → 4)
- Standalone pages for intro, hardware, weekly-breakdown

**Internal Linking**:
- Use relative links: `[ROS 2 Architecture](./ros2-architecture)` within same module
- Use absolute links: `[URDF in Module 1](/module1/urdf-for-humanoids)` across modules
- Link forward to related chapters: "We'll explore Isaac Sim in [Module 3](../module3/isaac-sim)"
- Link backward to prerequisites: "Refer to [ROS 2 Nodes](../module1/nodes-topics-services) for background"

**Breadcrumbs**:
- Docusaurus automatically generates breadcrumbs for nested structure
- Shows: Home > Module 1: ROS 2 > Nodes and Topics

---

## Research Decisions

### Decision 1: Content Depth
- **Decision**: Target **800-1,200 words per chapter**
- **Rationale**:
  - 800 words minimum for comprehensive coverage of concepts
  - 1,200 words maximum to maintain engagement and avoid overwhelming students
  - Allows for explanations, code examples, and diagrams without excessive length
  - Hackathon timeline constraint (20+ chapters = 16,000-24,000 total words, achievable in 16-20 hours with AI assistance)

---

### Decision 2: Database Schema
- **Decision**: Use **JSONB columns** for user backgrounds and conversation citations
- **Rationale**:
  - **Flexibility**: Questionnaire can evolve (add new background questions) without database migrations
  - **Simplicity**: Fewer tables to manage (vs. normalized user_programming_languages, user_hardware tables)
  - **PostgreSQL strength**: Neon Postgres fully supports JSONB with indexing and queries
  - **Agile hackathon timeline**: Avoid complex migrations, focus on functionality
- **Tradeoff**: Sacrifices strict validation at database level for application-level flexibility

---

### Decision 3: Token Storage
- **Decision**: Use **localStorage for hackathon development**, document **httpOnly cookies for production**
- **Rationale**:
  - **Development speed**: localStorage is simpler (no CORS credential configuration, easier debugging)
  - **Production security**: httpOnly cookies prevent XSS attacks, should be recommended for deployment
  - **Hackathon priority**: Demonstrate functionality > production-grade security
  - **Documentation**: Include security note in quickstart.md and README.md about production cookie usage

---

### Decision 4: Module Organization
- **Decision**: Use **nested directory structure** (module1/, module2/, module3/, module4/)
- **Rationale**:
  - **Scalability**: 20+ chapters organized into 4 modules (4-6 chapters each) prevents flat directory clutter
  - **Docusaurus support**: Built-in category sidebar with collapsible sections
  - **Mental model**: Students understand "I'm in Module 2, Chapter 3" rather than "I'm on page 8"
  - **Code examples**: Can co-locate code files, images, diagrams with chapters in module directories
- **Tradeoff**: Slightly more complex sidebar configuration, but better long-term organization

---

## Summary

This research phase has resolved key technical decisions for the Physical AI textbook project:

1. **Content**: 20+ chapters organized into 4 modules, 800-1,200 words each, covering ROS 2, Gazebo/Unity, NVIDIA Isaac, and VLA
2. **Database**: Neon Serverless Postgres with psycopg2, JSONB columns for flexible schema, connection pooling for performance
3. **Authentication**: Simplified JWT backend (python-jose, passlib), Better-auth for UI components, localStorage for tokens (httpOnly cookies for production)
4. **Structure**: Nested Docusaurus directories with collapsible sidebar categories, internal linking strategy for navigation

**Next Steps**: Proceed to Phase 1 (data-model.md, contracts/, quickstart.md) to formalize database schema, API contracts, and setup documentation.
