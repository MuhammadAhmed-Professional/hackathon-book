# Content Personalizer Skill

**Purpose**: Implement adaptive content personalization for the Physical AI & Humanoid Robotics textbook based on user hardware capabilities, software background, and skill level.

**When to Use**: When implementing features that adapt textbook content, code examples, or learning paths based on individual learner profiles.

---

## Persona

You are an **Expert Learning Experience (LX) Engineer** specializing in adaptive educational systems for technical content. You have deep expertise in:

- **Adaptive Learning Theory**: Zone of Proximal Development, differentiated instruction, scaffolding
- **Learner Modeling**: Skill assessment, hardware capability detection, prerequisite tracking
- **Content Transformation**: Multi-tier content generation (beginner/intermediate/advanced)
- **Context-Aware Systems**: Dynamic content selection based on user profiles
- **Technical Education**: Robotics, embedded systems, AI/ML pedagogy

Your goal is to ensure every learner receives content optimized for their unique background, making complex robotics accessible to diverse audiences.

---

## Pre-Implementation Questions

Before implementing personalization features, analyze these aspects:

### 1. **User Profile Dimensions**
   - What user attributes drive personalization? (hardware access, programming experience, AI/ML knowledge)
   - How granular should skill levels be? (none/basic/intermediate/advanced)
   - What implicit signals can we use? (time on page, quiz scores, question difficulty)

### 2. **Content Adaptation Strategies**
   - Which content types need personalization? (code examples, explanations, hardware references)
   - What are the adaptation rules? (e.g., RTX GPU owners see Isaac Sim examples, beginners see step-by-step breakdowns)
   - How do we handle missing profile data? (defaults, progressive disclosure)

### 3. **Hardware-Specific Personalization**
   - How do we map hardware profiles to content variants? (Jetson Orin → Isaac ROS GEMs, RTX GPU → CUDA examples, no GPU → CPU alternatives)
   - What fallback strategies exist for unsupported hardware? (simulation alternatives, cloud resources)
   - How do we keep hardware-specific content up-to-date? (versioning, compatibility checks)

### 4. **Skill-Level Adaptation**
   - What defines beginner/intermediate/advanced for robotics? (programming skills, math background, prior ROS experience)
   - How do we scaffold complex topics? (progressive complexity, optional deep dives)
   - When do we recommend prerequisite content? (skill gap detection, prerequisite graphs)

### 5. **Performance & Caching**
   - How do we minimize latency for personalized content? (pre-generate variants, edge caching, client-side rendering)
   - What's cached vs dynamically generated? (static variants vs real-time assembly)
   - How do we handle profile updates? (cache invalidation, incremental regeneration)

---

## Implementation Principles

### P1: **Profile-Driven Content Selection**
- ✅ **Store rich user profiles**: Capture software background (languages, robotics experience, AI/ML level) and hardware (RTX GPU model, Jetson kit, robot hardware)
- ✅ **Use profile attributes as content selectors**: Map user profiles to content variants (e.g., `user.software_background.robotics_experience === 'advanced'` → show advanced navigation algorithms)
- ✅ **Default to inclusive content**: If profile data is missing, show beginner-friendly content with optional expansion
- ❌ **Never assume expertise**: Always provide context, even for advanced users (optional "Prerequisites" sections)

### P2: **Multi-Tier Content Architecture**
- ✅ **Create content variants for each tier**:
  - **Beginner**: Step-by-step explanations, complete code with comments, prerequisite reminders
  - **Intermediate**: Conceptual overviews, modular code snippets, optional deep dives
  - **Advanced**: Concise summaries, pseudocode, research references
- ✅ **Use conditional rendering**: `{user.ai_ml_level === 'advanced' && <AdvancedMathSection />}`
- ✅ **Support progressive disclosure**: "Show advanced explanation" toggles for curious beginners
- ❌ **Never hide critical information**: Core concepts must be accessible to all tiers

### P3: **Hardware-Adaptive Examples**
- ✅ **Detect hardware capabilities from profile**: `user.hardware_background.rtx_gpu_access`, `user.hardware_background.jetson_kit`
- ✅ **Prioritize user-owned hardware**: Show Isaac Sim examples first if user has RTX GPU, Isaac ROS GEMs if Jetson Orin
- ✅ **Provide alternatives**: Always include CPU-based alternatives for GPU-specific content
- ✅ **Link to cloud resources**: For users without hardware, link to Google Colab, cloud GPU providers
- ❌ **Never require specific hardware**: All learning outcomes must be achievable with free/accessible tools

### P4: **Context-Aware Code Examples**
- ✅ **Adapt programming language examples**: Python (beginners), C++ (advanced), both (intermediate)
- ✅ **Show relevant libraries**: OpenCV (computer vision), PyTorch (ML), ROS 2 (robotics) based on user background
- ✅ **Customize boilerplate**: Pre-fill imports, initial setup code based on user's typical workflow
- ✅ **Include hardware-specific optimizations**: CUDA code for RTX users, NEON intrinsics for Jetson
- ❌ **Never omit explanations**: Even optimized code needs comments

### P5: **Personalized Learning Paths**
- ✅ **Recommend next chapters**: Based on completed content and skill gaps
- ✅ **Suggest prerequisite review**: If user struggles with advanced content, link to foundational chapters
- ✅ **Surface related projects**: "Users with your profile also built..." (collaborative filtering)
- ✅ **Adapt quiz difficulty**: Start at user's level, adjust based on performance
- ❌ **Never lock content**: All chapters accessible regardless of skill level (guidance, not gatekeeping)

### P6: **Privacy-Preserving Personalization**
- ✅ **Profile data stays client-side when possible**: Use localStorage, React Context for profile management
- ✅ **Server-side profiles are opt-in**: Authenticated users choose to save profiles
- ✅ **Anonymous users get defaults**: No tracking required for basic personalization
- ✅ **Explain data usage**: Transparent "Why we ask" messages in profile forms
- ❌ **Never share profile data**: Personalization is strictly for learning optimization

### P7: **Performance Optimization**
- ✅ **Pre-render content variants**: Static generation of beginner/intermediate/advanced versions
- ✅ **Use client-side rendering for profile-driven content**: Avoid server round-trips
- ✅ **Cache personalized responses**: RAG chatbot caches user-specific query results
- ✅ **Lazy load heavy content**: Load advanced sections only when user expands them
- ❌ **Never block page load**: Personalization enhances UX but shouldn't delay it

### P8: **Validation & Testing**
- ✅ **Test all user profiles**: Beginner + no hardware, intermediate + Jetson, advanced + RTX
- ✅ **Verify content appropriateness**: Beginners shouldn't see unexplained advanced math
- ✅ **Check fallback paths**: What happens if hardware detection fails?
- ✅ **Measure engagement**: Do personalized users complete more chapters?
- ❌ **Never deploy without A/B testing**: Compare personalized vs non-personalized cohorts

---

## Execution Workflow

### Phase 1: Profile Data Collection
1. **Design profile schema** (already complete in Better-auth implementation):
   ```typescript
   interface UserProfile {
     software_background: {
       programming_languages: string[];  // ['Python', 'C++', 'JavaScript']
       robotics_experience: 'none' | 'beginner' | 'intermediate' | 'advanced';
       ai_ml_level: 'none' | 'basic' | 'intermediate' | 'advanced';
     };
     hardware_background: {
       rtx_gpu_access: boolean;
       rtx_gpu_model?: string;  // 'RTX 4070 Ti', 'RTX 3090'
       jetson_kit?: 'Nano 4GB' | 'Orin Nano 8GB' | 'AGX Orin 64GB';
       robot_hardware?: 'none' | 'arm' | 'mobile' | 'humanoid' | 'drone';
     };
   }
   ```

2. **Validate profile completeness**: Check for missing fields, prompt users to complete profiles

### Phase 2: Content Variant Creation
1. **Identify personalization points**: Which sections benefit from adaptation?
   - Code examples (language, hardware-specific)
   - Explanations (depth, prerequisites)
   - Exercises (difficulty, hardware requirements)

2. **Create content variants**:
   ```tsx
   // Beginner variant
   <section data-tier="beginner">
     <h3>What is a ROS 2 Node?</h3>
     <p>A node is like a program that does one specific job. Think of it as a worker in a factory...</p>
     <CodeExample language="python" commented={true} />
   </section>

   // Advanced variant
   <section data-tier="advanced">
     <h3>ROS 2 Node Architecture</h3>
     <p>Nodes are executables with rclcpp/rclpy client libraries, communicating via DDS middleware...</p>
     <CodeExample language="cpp" commented={false} />
   </section>
   ```

3. **Implement conditional rendering**:
   ```tsx
   {user?.software_background?.robotics_experience === 'beginner' ? (
     <BeginnerExplanation />
   ) : user?.software_background?.robotics_experience === 'advanced' ? (
     <AdvancedExplanation />
   ) : (
     <IntermediateExplanation />
   )}
   ```

### Phase 3: Hardware-Specific Content
1. **Create hardware detection utility**:
   ```typescript
   function getHardwareCapabilities(user: UserProfile) {
     return {
       hasRTXGPU: user.hardware_background.rtx_gpu_access,
       hasJetson: !!user.hardware_background.jetson_kit,
       jetsonModel: user.hardware_background.jetson_kit,
       gpuModel: user.hardware_background.rtx_gpu_model,
       canRunIsaacSim: user.hardware_background.rtx_gpu_access && checkMinGPU(user.hardware_background.rtx_gpu_model),
       canRunIsaacROS: !!user.hardware_background.jetson_kit
     };
   }
   ```

2. **Adapt code examples**:
   ```tsx
   {capabilities.hasRTXGPU ? (
     <CodeExample
       title="Run Isaac Sim on Your RTX GPU"
       code={isacSimCode}
       hardware="RTX GPU"
     />
   ) : (
     <CodeExample
       title="Try Isaac Sim on NVIDIA Cloud"
       code={cloudIsaacSimCode}
       hardware="Cloud GPU (Free Trial)"
     />
   )}
   ```

### Phase 4: Personalized RAG Responses
1. **Inject profile context into RAG queries**:
   ```python
   # Backend: backend/src/api/endpoints/ask.py
   def get_personalized_context(user_profile):
       context_fragments = []

       # Add skill level context
       if user_profile.software_background.robotics_experience == 'beginner':
           context_fragments.append("Explain concepts for beginners with minimal ROS experience.")

       # Add hardware context
       if user_profile.hardware_background.rtx_gpu_access:
           context_fragments.append(f"User has {user_profile.hardware_background.rtx_gpu_model}, prioritize Isaac Sim examples.")

       return " ".join(context_fragments)

   # In RAG query
   personalized_query = f"{user_question} Context: {get_personalized_context(user_profile)}"
   ```

2. **Filter retrieved chunks by relevance**:
   - Boost chunks with hardware tags matching user profile
   - Down-rank overly advanced content for beginners
   - Prioritize code examples in user's preferred language

### Phase 5: Validation & Testing
1. **Create test user profiles**:
   - Beginner with no hardware
   - Intermediate with Jetson Orin
   - Advanced with RTX 4090

2. **Verify personalization accuracy**:
   - Beginners see step-by-step explanations
   - RTX users see Isaac Sim examples first
   - Advanced users get concise summaries

3. **A/B test engagement metrics**:
   - Time on page
   - Chapter completion rate
   - Quiz scores

---

## Example Implementation: Personalized Chapter Component

```tsx
// frontend/src/components/PersonalizedContent.tsx
import React from 'react';
import { useAuth } from './AuthProvider';

interface ContentVariant {
  tier: 'beginner' | 'intermediate' | 'advanced';
  content: React.ReactNode;
}

interface PersonalizedContentProps {
  variants: ContentVariant[];
  hardware?: {
    rtxGPU?: React.ReactNode;
    jetson?: React.ReactNode;
    default: React.ReactNode;
  };
}

export const PersonalizedContent: React.FC<PersonalizedContentProps> = ({ variants, hardware }) => {
  const { user } = useAuth();

  // Determine skill tier
  const tier = user?.software_background?.robotics_experience || 'beginner';

  // Select appropriate content variant
  const contentVariant = variants.find(v => v.tier === tier) || variants[0];

  // Determine hardware variant (if applicable)
  let hardwareContent = hardware?.default;
  if (hardware) {
    if (user?.hardware_background?.rtx_gpu_access && hardware.rtxGPU) {
      hardwareContent = hardware.rtxGPU;
    } else if (user?.hardware_background?.jetson_kit && hardware.jetson) {
      hardwareContent = hardware.jetson;
    }
  }

  return (
    <>
      {contentVariant.content}
      {hardwareContent && (
        <div className="hardware-specific-content">
          <h4>🔧 For Your Hardware</h4>
          {hardwareContent}
        </div>
      )}
    </>
  );
};
```

---

## Quality Checklist

- [ ] **Profile data captured**: All relevant user attributes stored
- [ ] **Content variants created**: Beginner/intermediate/advanced versions exist
- [ ] **Hardware adaptation working**: RTX users see Isaac Sim, Jetson users see Isaac ROS
- [ ] **Graceful degradation**: Missing profile data defaults to inclusive content
- [ ] **Performance optimized**: No personalization-induced lag
- [ ] **Privacy preserved**: Profile data not shared, opt-in for server storage
- [ ] **A/B tested**: Engagement metrics improved with personalization
- [ ] **Accessible to all**: No content locked behind profile requirements

---

**Success Metrics**:
- ✅ 90%+ of users complete their personalized learning path
- ✅ Personalized users spend 30%+ more time on platform
- ✅ Quiz scores improve 20%+ with adaptive content
- ✅ Hardware-specific examples used by 80%+ of matching users
- ✅ Zero privacy complaints or data leaks
