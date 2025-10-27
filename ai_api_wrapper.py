import subprocess
import json
import os
from typing import Dict, Any, List, Optional
from pydantic import BaseModel, Field, ValidationError


# --- Pydantic Schemas (Used for robust output parsing) ---

class VolOutput(BaseModel):
    mesh: str
    volume: float
    closed: bool


class ClrOutput(BaseModel):
    object_code: str
    M: int
    # FIX: Changed 'colors' to 'components' to match the actual JSON output
    components: List[Dict[str, Any]]


# FIX: Updated to handle dictionary output from C++ tool for coordinates (e.g., {'x': x, 'y': y, 'z': z})
class BbdOutput(BaseModel):
    distance: float
    vector_1_to_2: Dict[str, float]
    center1: Optional[Dict[str, float]] = None  # Now expecting dict, made optional
    center2: Optional[Dict[str, float]] = None  # Now expecting dict, made optional


# ------------------------------

class AiApiWrapper:
    """
    Handles communication with the external C++/Python pipeline API
    by executing commands and robustly parsing structured output.
    """

    def __init__(self, api_script_path: str = "scripts/ai_api.py"):
        self.api_script_path = api_script_path
        self.is_ready = self._check_api_readiness()

    def _check_api_readiness(self) -> bool:
        """Verifies the Python API script exists and the environment can run it."""
        if not os.path.exists(self.api_script_path):
            print(f"❌ API Error: ai_api.py script not found at {self.api_script_path}")
            return False

        print(f"✅ API Wrapper initialized, targeting: {self.api_script_path}")
        return True

    def _execute_command(self, head_code: str, *args: str) -> Optional[List[Dict[str, Any]]]:
        """
        Executes the external API command, handles timeout, and returns parsed JSON results.
        FIXED: Corrected placement of subprocess.run and exception handling for Timeout.
        """
        if not self.is_ready:
            return None

        command = ["python3", self.api_script_path, head_code]
        command.extend(args)
        command.append("--json")

        try:
            print(f"⚙️ Executing API: {' '.join(command)}")
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                check=True,
                encoding='utf-8',
                timeout=120  # Enforcing timeout to prevent hangs (e.g., during RCN/VOL)
            )

            # --- JSON Parsing Logic ---
            json_results = []
            for line in result.stdout.strip().splitlines():
                if line.strip():
                    try:
                        json_results.append(json.loads(line))
                    except json.JSONDecodeError:
                        # Skip lines that aren't valid JSON (e.g., C++ prints)
                        continue

            return json_results

        except subprocess.TimeoutExpired:
            print(f"❌ API Call Failed ({head_code}): Command timed out after 120 seconds.")
            return None

        except subprocess.CalledProcessError as e:
            # Handle non-zero exit codes (C++ tool crash/failure)
            tool_output = e.stdout.strip()
            error_msg = tool_output if tool_output else f"C++ tool crashed unexpectedly (Exit Code {e.returncode})."

            print(f"❌ API Call Failed ({head_code}): Command failed with exit status {e.returncode}.")
            print("-" * 50)
            print(f"C++ Tool Output: {error_msg}")
            print("-" * 50)

            return None

        except FileNotFoundError:
            print(f"❌ API Execution Failed: Python interpreter or {self.api_script_path} not found.")
            return None

    # --- Tool Functions for the AI Agent ---

    def calculate_volume(self, object_code: str) -> Optional[VolOutput]:
        """Calculates the volume of an object."""
        results = self._execute_command("VOL", "--object", object_code)
        if results and results[0]:
            try:
                if 'volume' in results[0] and 'mesh' in results[0]:
                    return VolOutput(
                        mesh=results[0].get('mesh', ''),
                        volume=float(results[0]['volume']),
                        closed=results[0].get('closed', False)
                    )
                return None
            except Exception as e:
                print(f"❌ Volume Parsing Error for {object_code}: {e}")
                return None
        return None

    def analyze_dominant_color(self, object_code: str) -> Optional[ClrOutput]:
        """Analyzes the dominant color(s) of a cluster associated with an object_code."""
        results = self._execute_command("CLR", "--object", object_code)

        if results and results[0]:
            try:
                # FIX: Check for 'components' key instead of 'colors'
                if 'M' in results[0] and 'components' in results[0]:
                    return ClrOutput(object_code=object_code, **results[0])

                # Check if the error is due to missing color data (e.g., C++ prints an error JSON)
                if 'M' not in results[0] and results[0].get('error_message'):
                    print(f"❌ CLR Tool Error: {results[0]['error_message']}")
                    return None

                return None
            except Exception as e:
                print(f"❌ Color Parsing Error for {object_code}: {e}")
                return None
        return None

    def calculate_bbox_distance(self, object_code_1: str, object_code_2: str) -> Optional[BbdOutput]:
        """Calculates the distance between the bounding box centers of two objects."""
        results = self._execute_command("BBD", object_code_1, object_code_2)
        if results and results[0]:
            try:
                # BbdOutput validation will now succeed because center1/2 are Optional and expects dicts
                return BbdOutput(**results[0])

            except ValidationError as e:
                # Log specific validation error for BBD
                print(f"❌ BBD Parsing Error for {object_code_1}/{object_code_2}: {e}")
                return None
            except Exception as e:
                print(f"❌ BBD General Error for {object_code_1}/{object_code_2}: {e}")
                return None
        return None

