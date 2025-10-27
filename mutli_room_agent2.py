import sqlite3
import pandas as pd
import os
import base64
import re
import json
import textwrap
import subprocess  # Required for AiApiWrapper's functions
from typing import Dict, List, Optional, Any, Tuple
from openai import AzureOpenAI
from dotenv import load_dotenv
from PIL import Image
import io
import traceback
# Assuming ai_api_wrapper.py is in the same directory or accessible via PYTHONPATH
from ai_api_wrapper import AiApiWrapper

# Load environment variables
load_dotenv()


class FinalSpatialAIAgent:
    """A robust spatial AI agent integrated with an external C++/Python pipeline."""

    def __init__(self, database_path: str = "spatial_rooms.db", use_images: bool = False):
        print("🚀 Initializing Final Spatial AI Agent...")
        self.database_path = database_path
        self.use_images = use_images
        self.current_room_id = None
        self.room_cache = {}

        # ------------------ TOOL INITIALIZATION ------------------
        self.api_wrapper = AiApiWrapper(api_script_path="scripts/ai_api.py")
        if not self.api_wrapper.is_ready:
            print("⚠️ WARNING: External C++ API wrapper is NOT ready. Volume/Color/Distance queries will fail.")
        # ---------------------------------------------------------

        self.conn = sqlite3.connect(database_path)
        print(f"✅ Connected to database: {database_path}")

        try:
            self.client = AzureOpenAI(
                azure_endpoint="https://azure-openai-scanplan.openai.azure.com/",
                api_key=os.getenv("API_KEY"),
                api_version="2025-02-01-preview"
            )
            print("✅ OpenAI client initialized")
        except Exception as e:
            print(f"❌ OpenAI client initialization failed: {e}")
            raise Exception("OpenAI API is required for this agent.")

        # Load database structure with type conversion (CRITICAL for stability)
        self.floors_df = pd.read_sql_query("SELECT * FROM floors ORDER BY floor_number", self.conn)
        self.rooms_df = pd.read_sql_query("""
            SELECT r.*, f.floor_name, f.floor_number
            FROM rooms r 
            JOIN floors f ON r.floor_id = f.floor_id 
            ORDER BY f.floor_number, r.room_number
        """, self.conn)

        if not self.rooms_df.empty:
            for col in ['room_id', 'floor_id', 'floor_number']:
                if col in self.rooms_df.columns:
                    self.rooms_df[col] = self.rooms_df[col].astype(int)

        if not self.floors_df.empty:
            self.floors_df['floor_number'] = self.floors_df['floor_number'].astype(int)

        print(f"✅ Found {len(self.floors_df)} floors and {len(self.rooms_df)} rooms")

        # Set a default room that has objects
        rooms_with_objects_df = pd.read_sql_query("""
            SELECT r.room_id FROM rooms r 
            JOIN objects o ON r.room_id = o.room_id 
            GROUP BY r.room_id LIMIT 1
        """, self.conn)

        if not rooms_with_objects_df.empty:
            self.current_room_id = int(rooms_with_objects_df.iloc[0]['room_id'])
            room_info = self.rooms_df[self.rooms_df['room_id'] == self.current_room_id].iloc[0]
            print(f"✅ Auto-selected room with objects: {room_info['room_name']} on {room_info['floor_name']}")
        elif len(self.rooms_df) > 0:
            self.current_room_id = int(self.rooms_df.iloc[0]['room_id'])
            room_info = self.rooms_df.iloc[0]
            print(f"✅ Default room: {room_info['room_name']}")
        else:
            print("⚠ No rooms found in the database.")

        print("✅ Final Spatial AI Agent initialized")

    def _parse_room_reference(self, query: str) -> Optional[Tuple[str, int]]:
        """Parse room number (string) and floor number (int) from the query."""
        query_lower = query.lower()
        pattern1 = r'room[\s_]?(\d+)\s*(?:on\s+)?floor[\s_]?(\d+)'
        match = re.search(pattern1, query_lower)
        if match:
            room_num = match.group(1).zfill(3)
            floor_num = int(match.group(2))
            return (room_num, floor_num)

        pattern2 = r'in\s+room[\s_]?(\d+)'
        match = re.search(pattern2, query_lower)
        if match:
            room_num = match.group(1).zfill(3)
            if self.current_room_id:
                current_room = self.rooms_df[self.rooms_df['room_id'] == self.current_room_id]
                if not current_room.empty:
                    floor_num = current_room.iloc[0]['floor_number']
                    return (room_num, floor_num)
            return (room_num, 0)

        return None

    def _find_room_by_reference(self, room_num: str, floor_num: int) -> Optional[int]:
        """Find room_id by room number (str) and floor number (int)"""
        room_match = self.rooms_df[
            (self.rooms_df['room_number'] == room_num) &
            (self.rooms_df['floor_number'] == floor_num)
            ]

        if not room_match.empty:
            return int(room_match.iloc[0]['room_id'])

        return None

    def _find_object_code_by_nlp(self, room_id: int, query: str) -> Optional[str]:
        """
        NEW: Finds the object code of the first matching object class in the room,
        based on keywords in the user query.
        """
        room_summary = self.get_room_summary(room_id)
        if not room_summary:
            return None

        # Common object classes used as keywords (adjust based on your dataset)
        class_keywords = [
            'chair', 'table', 'window', 'door', 'shell', 'couch', 'plant',
            'monitor', 'curtain', 'sofa', 'desk', 'cabinet'
        ]

        target_class = None
        query_lower = query.lower()
        for keyword in class_keywords:
            if keyword in query_lower and 'total' not in query_lower:  # Avoid triggering on total counts
                target_class = keyword
                break

        if target_class:
            # Find the first object of that class in the room's inventory
            for obj in room_summary['objects']:
                if obj['class'].lower() == target_class:
                    print(f"🤖 Found candidate object via NLP: {obj['object_code']} (Class: {target_class})")
                    return obj['object_code']

        return None

    def get_room_summary(self, room_id: int) -> Dict:
        """Get room summary including object_code and planes data."""
        if room_id in self.room_cache:
            return self.room_cache[room_id]

        try:
            # 1. Room Info
            room = pd.read_sql_query("""
                SELECT 
                    r.room_id, r.room_name, r.room_number, r.room_type, f.floor_name, f.floor_number,
                    COALESCE(r.total_area, 0.0) as total_area, 
                    COALESCE(r.total_volume, 0.0) as total_volume,
                    COALESCE(r.length, 0.0) as length, COALESCE(r.width, 0.0) as width, COALESCE(r.height, 0.0) as height
                FROM rooms r JOIN floors f ON r.floor_id = f.floor_id WHERE r.room_id = ?
            """, self.conn, params=[room_id])

            if room.empty: return {}

            # 2. Objects with object_code (essential for external API)
            objects = pd.read_sql_query("""
                SELECT 
                    class, object_code, obj_name, center_x, center_y, center_z, length, width, height 
                FROM objects
                WHERE room_id = ?
                ORDER BY class, object_code
            """, self.conn, params=[room_id])

            # 3. Planes (Walls/Floors)
            planes = pd.read_sql_query("""
                SELECT 
                    plane_class, area, normal_x, normal_y, normal_z 
                FROM planes
                WHERE room_id = ?
                ORDER BY plane_class, area DESC
            """, self.conn, params=[room_id])

            # 4. Images
            images = pd.read_sql_query("""
                SELECT image_id, image_name, image_path 
                FROM images 
                WHERE room_id = ?
                ORDER BY image_id
            """, self.conn, params=[room_id])

            result = {
                "room": room.iloc[0].to_dict(),
                "objects": objects.to_dict('records'),
                "planes": planes.to_dict('records'),
                "images": images.to_dict('records')
            }

            self.room_cache[room_id] = result
            return result

        except Exception as e:
            print(f"❌ Error during database query for room ID {room_id}: {e}")
            traceback.print_exc()
            return {}

    # FIX: Modified to include plane statistics for multi-room queries
    def _get_all_rooms_data(self) -> List[Dict]:
        """Get data for all rooms, calculating plane statistics for comparative queries."""
        all_rooms_data = []
        for _, room_row in self.rooms_df.iterrows():
            room_id = room_row['room_id']
            room_summary = self.get_room_summary(room_id)
            if room_summary:
                # Add object count
                room_summary['room']['object_count'] = len(room_summary['objects'])

                # Calculate total wall plane area and count for multi-room analysis
                wall_planes = [p for p in room_summary['planes'] if p['plane_class'].lower() == 'wall']
                room_summary['room']['wall_count'] = len(wall_planes)
                room_summary['room']['wall_area_total'] = sum(p['area'] for p in wall_planes)

                all_rooms_data.append(room_summary)
        return all_rooms_data

    def _create_system_prompt(self, room_data: Dict | List[Dict]) -> str:
        """Create detailed system prompt with tool instructions and data summary."""

        prompt = """You are an advanced Spatial AI assistant analyzing architectural spaces.
MISSION: Provide precise, data-driven spatial analysis using geometric data, visual context, and specialized tools.

EXTERNAL TOOL ACCESS (Requires object_code, e.g., '0-1-5'):
1. **VOLUME (VOL)**: Get the reconstructed mesh volume of an object.
2. **COLOR (CLR)**: Determine the dominant visual color(s) of an object.
3. **DISTANCE (BBD)**: Calculate the center-to-center distance between two objects.

"""
        # --- Data Context Generation ---
        if isinstance(room_data, dict):
            # Single room mode
            room = room_data.get('room', {})
            objects = room_data.get('objects', [])
            planes = room_data.get('planes', [])

            prompt += f"\nCURRENT ROOM: {room.get('room_name', 'Unknown')} on {room.get('floor_name', 'Unknown')}\n"
            prompt += f"Area: {room.get('total_area', 0.0):.2f}m², Dimensions: L:{room.get('length', 0.0):.2f}m x W:{room.get('width', 0.0):.2f}m\n\n"

            if objects:
                prompt += "COMPLETE OBJECT INVENTORY (IDs for Tool Use):\n"
                prompt += "=" * 50 + "\n"

                objects_by_class = {}
                for obj in objects:
                    class_name = obj['class']
                    if class_name not in objects_by_class: objects_by_class[class_name] = []
                    objects_by_class[class_name].append(obj)

                for class_name, class_objects in objects_by_class.items():
                    prompt += f"📦 {class_name.upper()} ({len(class_objects)} total):\n"
                    for obj in class_objects:
                        dims = []
                        if obj.get('length') is not None: dims.append(f"L:{obj['length']:.2f}m")
                        if obj.get('width') is not None: dims.append(f"W:{obj['width']:.2f}m")
                        if obj.get('height') is not None: dims.append(f"H:{obj['height']:.2f}m")

                        # CRITICAL: Include object_code in inventory
                        prompt += f"  • {obj['obj_name']} (CODE: {obj['object_code']}) [{' / '.join(dims)}] @({obj.get('center_x', 0.0):.1f}, {obj.get('center_y', 0.0):.1f})\n"
                prompt += "\n"

            if planes:
                prompt += "GEOMETRIC PLANE DATA (Walls, Floor, Ceiling):\n"
                plane_summary = {}
                for plane in planes:
                    class_name = plane['plane_class']
                    area = plane['area']
                    plane_summary[class_name] = plane_summary.get(class_name, 0) + area
                for cls, area in plane_summary.items():
                    prompt += f"  • {cls.upper()}: Total Area {area:.2f}m²\n"
                prompt += "\n"

        else:
            # Multi-room mode summary
            prompt += "MULTI-ROOM COMPARATIVE ANALYSIS:\n"
            prompt += "═" * 50 + "\n"

            # --- Provide aggregated plane data for multi-room analysis ---
            for i, rd in enumerate(room_data, 1):
                room = rd['room']
                objects = rd['objects']

                wall_info = f"Walls: {room.get('wall_count', 0)} planes, Total Area: {room.get('wall_area_total', 0.0):.2f}m²"
                object_summary = ", ".join(set([obj['class'] for obj in objects[:5]]))

                prompt += f"{i}. 🏢 {room['room_name']} on {room['floor_name']} ({room['room_type']})\n"
                prompt += f"   - Objects: {room['object_count']} items. Top Types: {object_summary or 'None'}\n"
                prompt += f"   - Planes: {wall_info}\n\n"

            prompt += "Note: Tool execution is limited to single-room context. \n"

        # --- Tool Usage Instruction ---
        prompt += """
TOOL USAGE REQUIREMENT:
If the user asks a question about an object's **VOLUME**, **COLOR**, or **DISTANCE** between two specific objects, you MUST respond ONLY with the tool call, using the following format.

Format: TOOL: [HEAD_CODE] [object_code_1] [object_code_2] (where obj_2 is optional for VOL/CLR)
Example:
- User: "What is the color of object 0-2-3?"
- Response: TOOL: CLR 0-2-3

---
CRITICAL INSTRUCTION: If an 'EXTERNAL API RESULT' is present in the prompt, you MUST ignore the 'TOOL: ...' instruction above and provide the final answer using the result data immediately. 
RESPONSE GUIDELINES:
1. **NARRATIVE & DETAIL:** Provide a well-structured, detailed, and professional narrative.
2. **CALCULATION BREAKDOWN:** For any comparison or derived metric (e.g., total area, distance, difference), show the specific inputs (object codes, metrics) used to reach the conclusion.
3. **DO NOT ECHO THE TOOL COMMAND.**
"""
        return prompt

    def _needs_visual_analysis(self, query: str) -> bool:
        """Check if query benefits from visual image analysis"""
        visual_keywords = [
            'color', 'colour', 'appearance', 'look', 'style', 'design', 'decor', 'aesthetic',
            'finish', 'material', 'texture', 'lighting', 'ambiance', 'see', 'visible', 'describe',
            'visual', 'picture', 'photo', 'image', 'panorama'
        ]
        return any(keyword in query.lower() for keyword in visual_keywords)

    def _get_room_images(self, room_id: int) -> List[Dict]:
        """Load and process room images with enhanced debugging output."""
        try:
            images_df = pd.read_sql_query(
                "SELECT * FROM images WHERE room_id = ?",
                self.conn, params=[room_id]
            )

            processed = []
            for _, img_row in images_df.iterrows():
                image_path = img_row['image_path']
                if os.path.exists(image_path):
                    try:
                        with Image.open(image_path) as img:
                            if img.mode != 'RGB':
                                img = img.convert('RGB')
                            max_size = 1024
                            if max(img.size) > max_size:
                                img.thumbnail((max_size, max_size), Image.Resampling.LANCZOS)

                            buffer = io.BytesIO()
                            img.save(buffer, format='JPEG', quality=90)
                            image_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')

                            processed.append({
                                "base64": image_base64,
                                "name": img_row['image_name']
                            })
                    except Exception as e:
                        print(f"  ⚠ Error processing file {image_path}: {e}")
                else:
                    print(f"  ❌ Image file NOT FOUND at path: {image_path}")

            return processed
        except Exception as e:
            print(f"⚠ Error loading images from database: {e}")
            return []

    def _add_images_to_messages(self, messages: List[Dict], images: List[Dict]) -> List[Dict]:
        """
        Adds images to the LAST message in the list (which should be the user message).
        FIXED: Ensures the target message content is converted to the required list format.
        """
        if not images or not messages:
            return messages

        # Target the last message in the list (the user's message)
        user_message_index = len(messages) - 1
        user_message = messages[user_message_index]

        # Get the existing content (either a string or an existing list of dicts)
        user_content = user_message["content"]

        # Convert simple string content to the multimodal list format
        if isinstance(user_content, str):
            user_content = [{"type": "text", "text": user_content}]
        elif not isinstance(user_content, list):
            # Default fallback if content type is unexpected
            user_content = [{"type": "text", "text": str(user_content)}]

        # Append image dictionaries to the content list
        for img in images:
            user_content.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{img['base64']}",
                    "detail": "high"
                }
            })

        # Update the content of the user message object
        messages[user_message_index]["content"] = user_content
        return messages

    def _parse_and_execute_tool(self, user_query: str) -> Optional[str]:
        """
        Parses the user query for keywords and extracts object codes (or finds them via NLP)
        to construct a tool call. Returns the tool result string or None.
        """
        query_lower = user_query.lower()

        # Helper to find specific object codes
        codes = re.findall(r'([\d\-\w]+)', user_query)
        valid_codes = [c for c in codes if re.match(r'^\d+-\d+-\d+$', c)]

        # --- Tool Mandate Check ---
        tool_keywords = {
            'CLR': ['color', 'colour', 'dominant color'],
            'BBD': ['distance between', 'how far apart', 'separation'],
            'VOL': ['volume', 'mesh volume', 'closed volume']
        }

        # Determine which tool to prioritize
        tool_to_run = None
        for code, keywords in tool_keywords.items():
            if any(kw in query_lower for kw in keywords):
                tool_to_run = code
                break

        if tool_to_run:
            # Step 1: Resolve codes (use specific codes first, then try NLP)
            target_codes = valid_codes
            if not target_codes and not self.current_room_id is None:
                # Try to find a single object code using NLP
                code_from_nlp = self._find_object_code_by_nlp(self.current_room_id, user_query)
                if code_from_nlp:
                    target_codes = [code_from_nlp]
                    print(f"🔍 Resolved query to object code: {target_codes[0]}")
                else:
                    print(f"⚠️ Tool query failed: No specific object code found or resolvable via NLP.")
                    return None

            # Step 2: Execute the resolved tool command
            if tool_to_run == 'CLR' and len(target_codes) >= 1:
                print(f"🛠️ Detected CLR command. Executing for code: {target_codes[0]}")
                result = self.api_wrapper.analyze_dominant_color(target_codes[0])
                if result:
                    color_desc = f"Dominant colors ({result.M} found): "
                    for c in result.colors:
                        rgb = f"({int(c['mean_r'])},{int(c['mean_g'])},{int(c['mean_b'])})"
                        color_desc += f"[Weight: {c['weight']:.2f}, RGB: {rgb}] "
                    return f"EXTERNAL API RESULT (CLR {target_codes[0]}): {color_desc.strip()}"
                return f"EXTERNAL API RESULT (CLR {target_codes[0]}): Color analysis failed. (Likely missing color data in asset)."

            elif tool_to_run == 'BBD' and len(target_codes) >= 2:
                obj1, obj2 = target_codes[0], target_codes[1]
                print(f"🛠️ Detected BBD command. Executing for {obj1} and {obj2}")
                result = self.api_wrapper.calculate_bbox_distance(obj1, obj2)
                if result:
                    v = result.vector_1_to_2
                    vec_str = f"({v['x']:.3f}, {v['y']:.3f}, {v['z']:.3f})"
                    return f"EXTERNAL API RESULT (BBD {obj1} {obj2}): Distance is {result.distance:.3f}m, Vector: {vec_str}."
                return f"EXTERNAL API RESULT (BBD {obj1} {obj2}): Distance calculation failed."

            elif tool_to_run == 'VOL' and len(target_codes) >= 1:
                obj_code = target_codes[0]
                print(f"🛠️ Detected VOL command. Executing for code: {obj_code}")
                result = self.api_wrapper.calculate_volume(obj_code)
                if result:
                    return f"EXTERNAL API RESULT (VOL {obj_code}): Volume is {result.volume:.3f}m³, Mesh Closed: {result.closed}."
                return f"EXTERNAL API RESULT (VOL {obj_code}): Volume calculation failed."

        return None  # No tool command was initiated

    def query(self, user_query: str) -> Dict:
        """Robust query handling with automatic room detection and external tool use."""

        target_room_id = self.current_room_id
        is_multi_room = any(kw in user_query.lower() for kw in
                            ['all rooms', 'every room', 'compare', 'across rooms',
                             'which room', 'total', 'sum', 'building'])

        # 1. Parse room reference (same as before)
        if not is_multi_room:
            room_ref = self._parse_room_reference(user_query)
            if room_ref:
                room_num, floor_num = room_ref
                detected_room_id = self._find_room_by_reference(room_num, floor_num)

                if detected_room_id:
                    target_room_id = detected_room_id
                    self.current_room_id = detected_room_id
                    print(f"🔍 Switched context: room_{room_num} on floor_{floor_num} (ID: {detected_room_id})")
                else:
                    print(f"⚠ Could not find room reference room_{room_num} on floor_{floor_num}. Using current room.")

        if not target_room_id and not is_multi_room:
            return {"error": "No room selected and query is not multi-room.", "query": user_query}

        try:
            # 2. Get Data
            if is_multi_room:
                room_data = self._get_all_rooms_data()
                scope = f"multi_room_{len(room_data)}"
            else:
                room_data = self.get_room_summary(target_room_id)
                if not room_data or not room_data.get('room'):
                    return {"error": f"Room ID {target_room_id} not found or data corrupt.", "query": user_query}
                scope = f"room_{target_room_id}"

            # 3. Check and Execute External Tool (CRITICAL STEP)
            tool_result_text = self._parse_and_execute_tool(user_query)

            # 4. Initialize Messages and System Prompt
            system_prompt = self._create_system_prompt(room_data)
            # Initialize messages structure with system prompt and the user message placeholder
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_query}  # User message initialized with raw query
            ]
            user_content = user_query  # Start user content as raw query

            # 5. Handle Visuals and Tool Result Integration
            images_used = False
            room_images = []
            is_visual_query = self._needs_visual_analysis(user_query)

            # --- Tool Result Handling ---
            if tool_result_text:

                if "Color analysis failed" in tool_result_text and is_visual_query:
                    # CLR failed, initiating visual fallback for color/visual requests
                    print("⚠️ CLR failed, initiating visual fallback.")
                    if self.use_images and not is_multi_room:
                        room_images = self._get_room_images(target_room_id)
                        if room_images:
                            self._add_images_to_messages(messages, room_images)
                            images_used = True
                            user_content = f"[Tool Failure: CLR failed. Analyzing Image Instead.]\n\nUser Query: {user_query}"
                        else:
                            user_content = f"[Tool Failure: CLR failed. No images available.]\n\nUser Query: {user_query}"
                    else:
                        user_content = f"[Tool Failure: CLR failed. Visual analysis disabled/unavailable.]\n\nUser Query: {user_query}"

                elif "Color analysis failed" not in tool_result_text:
                    # Tool succeeded (VOL/BBD/CLR success)
                    user_content = f"{tool_result_text}\n\nUser Query: {user_query}"
                    print("🛠️ Tool result added to prompt for LLM processing.")

            # --- Pure Visual Query Handling (No Tool Was Run) ---
            elif is_visual_query and self.use_images and not is_multi_room:
                room_images = self._get_room_images(target_room_id)
                if room_images:
                    self._add_images_to_messages(messages, room_images)
                    images_used = True
                    # user_content remains the raw query

            # 6. Finalize message list by updating the user message content
            messages[-1]["content"] = user_content

            # 7. API Call
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=messages,
                temperature=0.1,
                max_tokens=2500
            )

            # 8. Build Result
            result = {
                "query": user_query,
                "response": response.choices[0].message.content,
                "scope": scope,
                "used_images": images_used,
                "images_count": len(room_images)
            }

            if not is_multi_room:
                room_info = room_data['room']
                result.update({
                    "room": room_info['room_name'],
                    "floor": room_info['floor_name'],
                    "room_id": target_room_id
                })

            return result

        except Exception as e:
            traceback.print_exc()
            return {"error": f"Query failed: {str(e)}", "query": user_query}

    def close(self):
        """Cleanup resources"""
        self.conn.close()
        print("✅ Database connection closed")


def demo_agent():
    """Demonstrate the final agent capabilities"""
    print("🎯 FINAL SPATIAL AI AGENT DEMO")
    print("═" * 60)

    try:
        agent = FinalSpatialAIAgent(use_images=True)

        current_room_info = agent.rooms_df[agent.rooms_df['room_id'] == agent.current_room_id].iloc[0]
        print(f"\nInitial Room Context: {current_room_info['room_name']} on {current_room_info['floor_name']}")
        print("—" * 60)

        # NOTE: Using object codes that likely exist based on the previous database logs
        test_queries = [
            # 1. CLR on WORKING ASSET (0-7-12 is couch, confirmed functional externally)
            "What is the dominant color of the couch 0-7-12 in room 7 floor 0? Provide the weight and RGB values.",

            # 2. BBD: Distance between two objects in room 2 floor 0 (Should succeed and synthesize)
            "What is the distance between the chair object 0-2-4 and the shell object 0-2-3 in room 2 floor 0?",

            # 3. VOL: Volume check using NLP (Should succeed and synthesize)
            "What is the volume of the window in room 2 floor 0?",

            # 4. COMPLEX VISUAL/GEOMETRIC QUERY (Image must attach)
            "In room 6 floor 0, what is the total wall area and what is the appearance of the curtains?",

            # 5. NEW: Distance between two tables in different rooms (Multi-room BBD)
            "What is the distance between object 0-4-3 and 0-5-5?",

            # 6. NEW: Total count and area of floor planes (Multi-room plane summary)
            "Compare the total count and area of all floor planes across all floors.",

            # 7. NEW: Finding smallest object of a class (Requires looking up sizes in inventory)
            "Which chair in room 3 floor 0 has the smallest width?",

            # 8. NEW: Combined Visual Fallback & Area Check
            "What is the appearance of the plant in room 1 floor 0, and what is its center (X, Y) coordinate?",

            # 9. NEW: CLR on another object (Curtain) (Tests CLR failure handling and visual fallback)
            "What is the color of the curtain in room 6 floor 0? Use the best method possible.",

            # 10. NEW: Comparative analysis for Windows
            "Which room has the highest number of windows?"
        ]

        for query in test_queries:
            print(f"\n{'=' * 80}")
            print(f"Q: {query}")
            print("—" * 80)

            result = agent.query(query)

            if "error" in result:
                print(f"❌ ERROR: {result['error']}")
                continue

            print(f"📝 SCOPE: {result['scope']}")
            if not result['scope'].startswith('multi'):
                print(f"📍 CONTEXT: {result['room']} on {result['floor']}")
            print(f"🖼️ IMAGES USED: {result['used_images']} ({result.get('images_count', 0)} available)")
            print(f"\n📋 RESPONSE:\n{textwrap.fill(result['response'], width=80)}\n")

        agent.close()

    except Exception as e:
        print(f"❌ Failed to run demo: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    demo_agent()
