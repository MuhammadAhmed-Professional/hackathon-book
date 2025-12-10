"""
Translation service for converting textbook content to multiple languages.
Preserves code blocks, technical terms, and educational context.
"""
import re
import hashlib
from typing import Dict, List, Optional
from openai import OpenAI
from src.config import config

# Technical glossary - terms to preserve in English
PRESERVE_TERMS = [
    "ROS 2", "DDS", "URDF", "SDF", "MJCF", "Gazebo", "Isaac Sim",
    "quaternion", "Euler angles", "NVIDIA", "Jetson", "RTX",
    "rclpy", "cv_bridge", "geometry_msgs", "sensor_msgs",
    "QoS", "RTPS", "DDS-RTPS", "Isaac ROS", "GEM",
    "Python", "C++", "JavaScript", "TypeScript",
    "FastAPI", "React", "Docusaurus", "pytest", "Jest",
    "API", "REST", "HTTP", "HTTPS", "WebSocket",
    "JWT", "OAuth", "CORS", "SSL", "TLS"
]

# Robotics terminology glossary (English → Urdu)
ROBOTICS_GLOSSARY = {
    "robot": "روبوٹ",
    "simulation": "سمولیشن",
    "node": "نوڈ",
    "topic": "موضوع",
    "publisher": "پبلشر",
    "subscriber": "سبسکرائبر",
    "sensor": "سینسر",
    "camera": "کیمرہ",
    "algorithm": "الگورتھم",
    "navigation": "نیویگیشن",
    "perception": "ادراک",
    "control": "کنٹرول",
    "planning": "منصوبہ بندی",
    "actuator": "ایکچویٹر",
    "motor": "موٹر",
    "wheel": "پہیہ",
    "arm": "بازو",
    "gripper": "پکڑنے والا",
    "joint": "جوڑ",
    "link": "لنک",
    "frame": "فریم",
    "coordinate": "نقطہ",
    "transformation": "تبدیلی",
    "rotation": "گردش",
    "translation": "منتقلی",
    "velocity": "رفتار",
    "acceleration": "سرعت",
    "force": "قوت",
    "torque": "ٹارک",
    "mass": "کمیت",
    "inertia": "جمود",
    "friction": "رگڑ",
    "gravity": "کشش ثقل",
    "physics": "طبیعیات",
    "dynamics": "حرکیات",
    "kinematics": "حرکی تحلیل",
    "trajectory": "راستہ",
    "path": "راہ",
    "obstacle": "رکاوٹ",
    "collision": "ٹکراؤ",
    "detection": "شناخت",
    "tracking": "تعاقب",
    "localization": "محل وقوع کا تعین",
    "mapping": "نقشہ سازی",
    "SLAM": "SLAM",
    "odometry": "اوڈومیٹری",
    "IMU": "IMU",
    "GPS": "GPS",
    "LiDAR": "LiDAR",
    "depth camera": "گہرائی کیمرہ",
    "RGB camera": "RGB کیمرہ",
    "point cloud": "نقاط کا گروپ",
    "image": "تصویر",
    "video": "ویڈیو",
    "stream": "سٹریم",
    "data": "ڈیٹا",
    "message": "پیغام",
    "service": "سروس",
    "action": "ایکشن",
    "parameter": "پیرامیٹر",
    "configuration": "تشکیل",
    "launch file": "لانچ فائل",
    "package": "پیکج",
    "workspace": "ورک اسپیس",
    "build": "بلڈ",
    "compile": "کمپائل",
    "install": "انسٹال",
    "dependency": "انحصار",
    "library": "لائبریری",
    "module": "ماڈیول",
    "function": "فنکشن",
    "class": "کلاس",
    "object": "آبجیکٹ",
    "variable": "متغیر",
    "array": "صف",
    "list": "فہرست",
    "dictionary": "ڈکشنری",
    "string": "سلسلہ",
    "integer": "عدد صحیح",
    "float": "اعشاریہ",
    "boolean": "بولین",
    "callback": "کال بیک",
    "loop": "لوپ",
    "condition": "شرط",
    "exception": "استثنیٰ",
    "error": "خرابی",
    "warning": "انتباہ",
    "debug": "ڈی بگ",
    "test": "ٹیسٹ",
    "unit test": "یونٹ ٹیسٹ",
    "integration test": "انٹیگریشن ٹیسٹ",
    "performance": "کارکردگی",
    "optimization": "بہتری",
    "real-time": "حقیقی وقت",
    "latency": "تاخیر",
    "throughput": "پیداوار",
    "bandwidth": "بینڈوتھ",
    "frequency": "تعدد",
    "rate": "شرح",
    "timestamp": "وقت کی مہر",
    "synchronization": "ہم وقت سازی",
    "thread": "تھریڈ",
    "process": "پراسیس",
    "CPU": "سی پی یو",
    "GPU": "جی پی یو",
    "memory": "میموری",
    "RAM": "ریم",
    "storage": "اسٹوریج",
    "disk": "ڈسک",
    "network": "نیٹ ورک",
    "protocol": "پروٹوکول",
    "middleware": "مڈل ویئر",
    "interface": "انٹرفیس",
    "API": "API",
    "SDK": "SDK",
    "framework": "فریم ورک",
    "platform": "پلیٹ فارم",
    "operating system": "آپریٹنگ سسٹم",
    "Linux": "لینکس",
    "Ubuntu": "ابنٹو",
    "terminal": "ٹرمینل",
    "command": "کمانڈ",
    "script": "اسکرپٹ",
    "file": "فائل",
    "directory": "ڈائریکٹری",
    "folder": "فولڈر",
    "path": "راہ",
    "environment variable": "ماحولیاتی متغیر",
    "virtual environment": "مجازی ماحول",
    "container": "کنٹینر",
    "Docker": "ڈاکر",
    "cloud": "کلاؤڈ",
    "server": "سرور",
    "client": "کلائنٹ",
    "database": "ڈیٹا بیس",
    "query": "استفسار",
    "vector": "ویکٹر",
    "embedding": "ایمبیڈنگ",
    "model": "ماڈل",
    "training": "تربیت",
    "inference": "استنباط",
    "neural network": "عصبی نیٹ ورک",
    "deep learning": "گہری تعلیم",
    "machine learning": "مشین لرننگ",
    "AI": "مصنوعی ذہانت",
    "computer vision": "کمپیوٹر ویژن",
    "natural language": "فطری زبان",
    "processing": "پروسیسنگ",
    "classification": "درجہ بندی",
    "regression": "رجعت",
    "clustering": "خوشہ بندی",
    "prediction": "پیش گوئی",
    "accuracy": "درستگی",
    "precision": "صحت",
    "recall": "یادداشت",
    "F1 score": "F1 اسکور",
    "loss function": "نقصان فنکشن",
    "optimizer": "بہتری کار",
    "gradient": "تدریج",
    "learning rate": "سیکھنے کی شرح",
    "epoch": "دور",
    "batch": "بیچ",
    "dataset": "ڈیٹا سیٹ",
    "validation": "توثیق",
    "testing": "جانچ",
    "benchmark": "معیار"
}

class TranslationService:
    """Service for translating technical content while preserving code and terminology."""

    def __init__(self):
        """Initialize translation service with OpenAI client."""
        self.client = OpenAI(api_key=config.OPENAI_API_KEY)
        self.translation_cache: Dict[str, str] = {}

    def _extract_code_blocks(self, content: str) -> tuple[str, List[str]]:
        """
        Extract code blocks from content and replace with placeholders.

        Args:
            content: Original markdown content

        Returns:
            Tuple of (content with placeholders, list of extracted code blocks)
        """
        code_blocks = []
        code_pattern = r'```[\s\S]*?```'

        def replace_code(match):
            code_blocks.append(match.group(0))
            return f"[CODE_BLOCK_{len(code_blocks)-1}]"

        processed_content = re.sub(code_pattern, replace_code, content)
        return processed_content, code_blocks

    def _restore_code_blocks(self, content: str, code_blocks: List[str]) -> str:
        """
        Restore code blocks to translated content.

        Args:
            content: Translated content with placeholders
            code_blocks: List of original code blocks

        Returns:
            Content with code blocks restored
        """
        for i, block in enumerate(code_blocks):
            content = content.replace(f"[CODE_BLOCK_{i}]", block)
        return content

    def _generate_cache_key(self, content: str, target_language: str) -> str:
        """
        Generate cache key for translation.

        Args:
            content: Content to translate
            target_language: Target language code

        Returns:
            Cache key hash
        """
        key_string = f"{content}:{target_language}"
        return hashlib.md5(key_string.encode()).hexdigest()

    def translate(
        self,
        content: str,
        target_language: str = "ur",
        preserve_code: bool = True,
        use_cache: bool = True
    ) -> Dict[str, any]:
        """
        Translate content to target language while preserving technical context.

        Args:
            content: Text content to translate
            target_language: Target language code (ur=Urdu, es=Spanish, fr=French, etc.)
            preserve_code: Whether to preserve code blocks
            use_cache: Whether to use translation cache

        Returns:
            Dict with translated content, glossary, and metadata
        """
        # Check cache first
        cache_key = self._generate_cache_key(content, target_language)
        if use_cache and cache_key in self.translation_cache:
            return {
                "translated": self.translation_cache[cache_key],
                "source_language": "en",
                "target_language": target_language,
                "cached": True
            }

        # Extract code blocks if preservation is enabled
        code_blocks = []
        if preserve_code:
            content, code_blocks = self._extract_code_blocks(content)

        # Get language name
        language_names = {
            "ur": "Urdu",
            "es": "Spanish",
            "fr": "French",
            "de": "German",
            "zh": "Chinese",
            "ar": "Arabic",
            "ja": "Japanese",
            "ko": "Korean",
            "pt": "Portuguese",
            "ru": "Russian"
        }
        language_name = language_names.get(target_language, "Urdu")

        # Build translation prompt
        preserve_terms_str = ", ".join(PRESERVE_TERMS[:20])  # First 20 terms

        prompt = f"""Translate the following technical robotics content from English to {language_name}.

CRITICAL RULES:
1. PRESERVE these technical terms in English: {preserve_terms_str}
2. NEVER translate code-related terms: rclpy, cv_bridge, geometry_msgs, etc.
3. NEVER translate code placeholders like [CODE_BLOCK_0]
4. Translate explanations, learning objectives, and instructional text
5. Maintain markdown formatting (headings, lists, emphasis)
6. Use {language_name} robotics terminology from the glossary when available
7. Add English terms in parentheses after first use of technical terms
8. Keep URLs, file paths, and environment variables unchanged

For Urdu specifically:
- Use proper Urdu script (اردو)
- Keep technical English terms in Latin script within Urdu text
- Example: "ROS 2 نوڈز" (ROS 2 nodes in Urdu)

Content to translate:
{content}

Translate naturally while preserving technical accuracy. Output only the translated content."""

        try:
            # Call OpenAI for translation
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {
                        "role": "system",
                        "content": f"You are an expert technical translator specializing in robotics and AI education. Translate to {language_name} while preserving technical terminology, code blocks, and educational clarity."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                temperature=0.3,  # Lower temperature for more consistent translations
                max_tokens=4000
            )

            translated_content = response.choices[0].message.content.strip()

            # Restore code blocks
            if preserve_code and code_blocks:
                translated_content = self._restore_code_blocks(translated_content, code_blocks)

            # Cache the translation
            if use_cache:
                self.translation_cache[cache_key] = translated_content

            return {
                "translated": translated_content,
                "source_language": "en",
                "target_language": target_language,
                "language_name": language_name,
                "cached": False,
                "tokens_used": response.usage.total_tokens,
                "glossary": self._extract_glossary(content, target_language)
            }

        except Exception as e:
            raise Exception(f"Translation failed: {str(e)}")

    def _extract_glossary(self, content: str, target_language: str) -> List[Dict[str, str]]:
        """
        Extract glossary of technical terms from content.

        Args:
            content: Original English content
            target_language: Target language code

        Returns:
            List of glossary entries with English term, translation, and definition
        """
        if target_language != "ur":
            return []  # Only Urdu glossary implemented for now

        glossary = []
        for en_term, ur_term in ROBOTICS_GLOSSARY.items():
            if en_term.lower() in content.lower():
                glossary.append({
                    "english": en_term,
                    "translated": ur_term,
                    "preserve": en_term in PRESERVE_TERMS
                })

        # Limit to top 20 most relevant terms
        return glossary[:20]

    def get_supported_languages(self) -> List[Dict[str, str]]:
        """
        Get list of supported languages for translation.

        Returns:
            List of language dicts with code, name, and native name
        """
        return [
            {"code": "ur", "name": "Urdu", "native": "اردو", "rtl": True},
            {"code": "es", "name": "Spanish", "native": "Español", "rtl": False},
            {"code": "fr", "name": "French", "native": "Français", "rtl": False},
            {"code": "de", "name": "German", "native": "Deutsch", "rtl": False},
            {"code": "zh", "name": "Chinese", "native": "中文", "rtl": False},
            {"code": "ar", "name": "Arabic", "native": "العربية", "rtl": True},
            {"code": "ja", "name": "Japanese", "native": "日本語", "rtl": False},
            {"code": "ko", "name": "Korean", "native": "한국어", "rtl": False},
            {"code": "pt", "name": "Portuguese", "native": "Português", "rtl": False},
            {"code": "ru", "name": "Russian", "native": "Русский", "rtl": False}
        ]

# Global translation service instance
translation_service = TranslationService()
