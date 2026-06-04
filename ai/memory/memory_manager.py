"""
ai/memory/memory_manager.py
===========================
Handles local JSON-based persistent memory and FAISS vector search indexing for ARCHER.
Persists conversations, semantic locations, user preferences, experiences, and visual observations.
"""

import os
import json
import datetime
import logging
import threading
from pathlib import Path
import re

import numpy as np

# Optional FAISS import
try:
    import faiss
    FAISS_AVAILABLE = True
except ImportError:
    FAISS_AVAILABLE = False

logger = logging.getLogger("archer.memory")

class MemoryManager:
    def __init__(self, db_path: str = None) -> None:
        if db_path is None:
            project_root = Path(__file__).parent.parent.parent
            db_dir = project_root / "ai" / "memory" / "db"
            db_dir.mkdir(parents=True, exist_ok=True)
            self.db_path = str(db_dir / "memory.json")
            self.index_path = str(db_dir / "memory.index")
        else:
            self.db_path = db_path
            self.index_path = ":memory:"
            
        self._lock = threading.RLock()
        self.db = {
            "conversations": [],
            "locations": [],
            "experiences": [],
            "preferences": [],
            "visual_memory": [],
            "faiss_mapping": []  # List of {"faiss_index": i, "table_name": str, "db_id": int}
        }
        
        self._load_db()
        
        self._use_faiss = FAISS_AVAILABLE
        self._index = None
        self._init_faiss()

    def _load_db(self):
        with self._lock:
            if os.path.exists(self.db_path):
                try:
                    with open(self.db_path, "r", encoding="utf-8") as f:
                        data = json.load(f)
                        # Merge data to ensure all keys exist
                        for k in self.db.keys():
                            if k in data:
                                self.db[k] = data[k]
                except Exception as e:
                    logger.error(f"Failed to load JSON DB: {e}. Starting fresh.")
            else:
                self._save_db()

    def _save_db(self):
        with self._lock:
            try:
                with open(self.db_path, "w", encoding="utf-8") as f:
                    json.dump(self.db, f, indent=2)
            except Exception as e:
                logger.error(f"Failed to save JSON DB: {e}")

    def _get_next_id(self, table: str) -> int:
        with self._lock:
            items = self.db.get(table, [])
            if not items:
                return 1
            return max(item.get("id", 0) for item in items) + 1

    def _init_faiss(self) -> None:
        if not self._use_faiss:
            logger.warning("FAISS not installed or unavailable. Using exact term matching fallback.")
            return

        try:
            self._index = faiss.IndexFlatIP(384)
            if self.index_path != ":memory:" and os.path.exists(self.index_path):
                self._index = faiss.read_index(self.index_path)
                logger.info(f"Loaded existing FAISS index from {self.index_path}")
            else:
                self.rebuild_faiss_index()
        except Exception as e:
            logger.error(f"Error initializing FAISS: {e}. Falling back to exact matching.")
            self._use_faiss = False

    def rebuild_faiss_index(self) -> None:
        if not self._use_faiss:
            return
        
        with self._lock:
            try:
                self._index = faiss.IndexFlatIP(384)
                self.db["faiss_mapping"] = []
                faiss_idx = 0
                
                for table in ["preferences", "conversations", "experiences", "visual_memory"]:
                    for item in self.db[table]:
                        text = ""
                        if table == "conversations": text = item.get("text", "")
                        elif table == "experiences": text = item.get("description", "")
                        elif table == "visual_memory": text = item.get("scene_description", "")
                        elif table == "preferences": text = item.get("value", "")
                        
                        db_id = item.get("id", 0)
                        
                        vec = self._get_text_embedding(text)
                        self._index.add(np.expand_dims(vec, axis=0).astype('float32'))
                        
                        self.db["faiss_mapping"].append({
                            "faiss_index": faiss_idx,
                            "table_name": table,
                            "db_id": db_id
                        })
                        faiss_idx += 1
                        
                self._save_db()
                    
                if self.index_path != ":memory:":
                    faiss.write_index(self._index, self.index_path)
                logger.info("Successfully rebuilt and synchronized FAISS vector index.")
            except Exception as e:
                logger.error(f"Failed to rebuild FAISS index: {e}")
                self._use_faiss = False

    def _get_text_embedding(self, text: str) -> np.ndarray:
        np.random.seed(hash(text) % (2**32 - 1))
        vec = np.random.randn(384)
        norm = np.linalg.norm(vec)
        if norm > 0:
            vec = vec / norm
        return vec

    # --- Store APIs ---
    def store_conversation(self, speaker: str, text: str) -> None:
        with self._lock:
            db_id = self._get_next_id("conversations")
            timestamp = datetime.datetime.now().isoformat()
            self.db["conversations"].append({
                "id": db_id,
                "timestamp": timestamp,
                "speaker": speaker,
                "text": text,
                "importance": 0.5,
                "last_retrieved": timestamp,
                "access_count": 0
            })
            self._save_db()
            self._sync_vector_addition("conversations", db_id, text)

    def store_location(self, name: str, x: float, y: float, z: float, friendly_name: str) -> None:
        with self._lock:
            timestamp = datetime.datetime.now().isoformat()
            importance = 0.8
            db_id = None
            
            for loc in self.db["locations"]:
                if loc.get("name") == name:
                    loc["x"] = x
                    loc["y"] = y
                    loc["z"] = z
                    loc["friendly_name"] = friendly_name
                    loc["visit_count"] = loc.get("visit_count", 0) + 1
                    loc["last_retrieved"] = timestamp
                    db_id = loc["id"]
                    break
            else:
                db_id = self._get_next_id("locations")
                self.db["locations"].append({
                    "id": db_id,
                    "name": name,
                    "x": x,
                    "y": y,
                    "z": z,
                    "friendly_name": friendly_name,
                    "visit_count": 1,
                    "importance": importance,
                    "last_retrieved": timestamp,
                    "access_count": 0
                })
            
            self._save_db()
            self._sync_vector_addition("locations", db_id, f"Location {friendly_name} is located at {x}, {y}.")

    def store_experience(self, event_type: str, description: str) -> None:
        with self._lock:
            db_id = self._get_next_id("experiences")
            timestamp = datetime.datetime.now().isoformat()
            importance = 0.95 if "estop" in event_type.lower() or "fail" in event_type.lower() else 0.5
            
            self.db["experiences"].append({
                "id": db_id,
                "timestamp": timestamp,
                "event_type": event_type,
                "description": description,
                "importance": importance,
                "last_retrieved": timestamp,
                "access_count": 0
            })
            self._save_db()
            self._sync_vector_addition("experiences", db_id, description)

    def store_preference(self, key: str, value: str) -> None:
        with self._lock:
            timestamp = datetime.datetime.now().isoformat()
            importance = 0.9
            db_id = None
            
            for pref in self.db["preferences"]:
                if pref.get("key") == key:
                    pref["value"] = value
                    pref["last_retrieved"] = timestamp
                    db_id = pref["id"]
                    break
            else:
                db_id = self._get_next_id("preferences")
                self.db["preferences"].append({
                    "id": db_id,
                    "key": key,
                    "value": value,
                    "importance": importance,
                    "last_retrieved": timestamp,
                    "access_count": 0
                })
                
            self._save_db()
            self._sync_vector_addition("preferences", db_id, f"Preference {key}: {value}")

    def store_visual_observation(self, object_name: str, location: str, coords: list, ocr_text: str, scene_desc: str) -> None:
        with self._lock:
            db_id = self._get_next_id("visual_memory")
            timestamp = datetime.datetime.now().isoformat()
            importance = 0.7 if ocr_text else 0.6
            coords_str = ",".join([str(c) for c in coords]) if coords else "0,0,0"
            
            self.db["visual_memory"].append({
                "id": db_id,
                "timestamp": timestamp,
                "object_name": object_name,
                "location": location,
                "coordinates": coords_str,
                "ocr_text": ocr_text,
                "scene_description": scene_desc,
                "importance": importance,
                "last_retrieved": timestamp,
                "access_count": 0
            })
            self._save_db()
            self._sync_vector_addition("visual_memory", db_id, f"Saw {object_name} at {location}. {scene_desc} {ocr_text}")
            logger.info(f"Visual memory logged: Saw '{object_name}' in {location}.")

    def _sync_vector_addition(self, table_name: str, db_id: int, text: str) -> None:
        if not self._use_faiss or self._index is None:
            return
        with self._lock:
            try:
                vec = self._get_text_embedding(text)
                self._index.add(np.expand_dims(vec, axis=0).astype('float32'))
                
                faiss_idx = self._index.ntotal - 1
                self.db["faiss_mapping"].append({
                    "faiss_index": faiss_idx,
                    "table_name": table_name,
                    "db_id": db_id
                })
                self._save_db()
                
                if self.index_path != ":memory:":
                    faiss.write_index(self._index, self.index_path)
            except Exception as e:
                logger.error(f"FAISS sync failed: {e}")

    def _get_item_by_id(self, table: str, db_id: int):
        for item in self.db.get(table, []):
            if item.get("id") == db_id:
                return item
        return None

    # --- Query API ---
    def retrieve_context(self, user_query: str) -> str:
        clean_query = re.sub(r'[^\w\s]', '', user_query).lower()
        words = [w.strip() for w in clean_query.split() if len(w) >= 3]
        if not words:
            words = [user_query.strip().lower()]

        candidate_records = []

        with self._lock:
            # 1. FAISS Vector Search
            if self._use_faiss and self._index is not None and self._index.ntotal > 0:
                try:
                    q_vec = self._get_text_embedding(user_query)
                    D, I = self._index.search(np.expand_dims(q_vec, axis=0).astype('float32'), 10)
                    
                    for score, f_idx in zip(D[0], I[0]):
                        if f_idx < 0:
                            continue
                        
                        map_entry = next((m for m in self.db["faiss_mapping"] if m.get("faiss_index") == int(f_idx)), None)
                        if map_entry:
                            tbl = map_entry["table_name"]
                            db_id = map_entry["db_id"]
                            item = self._get_item_by_id(tbl, db_id)
                            
                            if item:
                                content_col = "text" if tbl == "conversations" else "description" if tbl == "experiences" else "scene_description" if tbl == "visual_memory" else "value"
                                rec_text = item.get(content_col, "")
                                imp = item.get("importance", 0.5)
                                last_ret = item.get("last_retrieved")
                                acc = item.get("access_count", 0)
                                
                                dt_hours = 0.0
                                if last_ret:
                                    try:
                                        last_dt = datetime.datetime.fromisoformat(last_ret)
                                        dt_hours = (datetime.datetime.now() - last_dt).total_seconds() / 3600.0
                                    except: pass
                                
                                recency_decay = np.exp(-0.02 * dt_hours)
                                final_score = (0.5 * float(score)) + (0.3 * imp) + (0.2 * recency_decay)
                                
                                candidate_records.append({
                                    "text": rec_text,
                                    "score": final_score,
                                    "table": tbl,
                                    "id": db_id,
                                    "access_count": acc
                                })
                except Exception as e:
                    logger.error(f"FAISS query lookup failed: {e}. Falling back to keywords.")
                    
            # 2. Keyword Search
            if not candidate_records:
                for word in words:
                    for item in self.db["preferences"]:
                        if word in str(item.get("key", "")).lower() or word in str(item.get("value", "")).lower():
                            candidate_records.append({"text": f"User preference - {item['key']}: {item['value']}", "score": 0.8, "table": "preferences", "id": item["id"], "access_count": item.get("access_count", 0)})
                    
                    for item in self.db["locations"]:
                        if word in str(item.get("name", "")).lower() or word in str(item.get("friendly_name", "")).lower():
                            candidate_records.append({"text": f"Location '{item['friendly_name']}' is mapped at coordinates [{item['x']}, {item['y']}]. Visited {item.get('visit_count',0)} times.", "score": 0.75, "table": "locations", "id": item["id"], "access_count": item.get("access_count", 0)})
                            
                    for item in reversed(self.db["experiences"]): # recent first
                        if word in str(item.get("description", "")).lower():
                            dt = item.get("timestamp", "").split("T")[0]
                            candidate_records.append({"text": f"[{dt}] Event ({item['event_type']}): {item['description']}", "score": 0.7, "table": "experiences", "id": item["id"], "access_count": item.get("access_count", 0)})
                            if len(candidate_records) > 10: break

            # 3. Visual Memory
            visual_candidates = []
            for word in words:
                for item in reversed(self.db["visual_memory"]):
                    if word in str(item.get("object_name", "")).lower() or word in str(item.get("location", "")).lower() or word in str(item.get("ocr_text", "")).lower() or word in str(item.get("scene_description", "")).lower():
                        dt = item.get("timestamp", "").split("T")[0]
                        ocr_part = f" (Labels read: '{item['ocr_text']}')" if item.get('ocr_text') else ""
                        desc = f"[{dt}] Visual Sighting: Saw a '{item.get('object_name')}' in the {item.get('location')} at coordinates [{item.get('coordinates')}]. {item.get('scene_description')}{ocr_part}"
                        visual_candidates.append({
                            "text": desc,
                            "score": 0.9,
                            "table": "visual_memory",
                            "id": item["id"],
                            "access_count": item.get("access_count", 0)
                        })
                        if len(visual_candidates) >= 3: break

            all_candidates = visual_candidates + candidate_records
            all_candidates = sorted(all_candidates, key=lambda x: x["score"], reverse=True)
            
            seen = set()
            unique_candidates = []
            for c in all_candidates:
                if c["text"] not in seen:
                    seen.add(c["text"])
                    unique_candidates.append(c)
                    
            top_candidates = unique_candidates[:5]
            
            # Promotion
            timestamp_now = datetime.datetime.now().isoformat()
            for c in top_candidates:
                item = self._get_item_by_id(c["table"], c["id"])
                if item:
                    item["access_count"] = item.get("access_count", 0) + 1
                    item["last_retrieved"] = timestamp_now
            self._save_db()

            context_parts = []
            if top_candidates:
                context_parts.append("### Retrieved Long-Term Memories & Visual Sightings:")
                context_parts.extend([f"- {c['text']}" for c in top_candidates])
                
            recent_convs = []
            conv_len = len(self.db["conversations"])
            for item in self.db["conversations"][max(0, conv_len-3):]:
                recent_convs.append(f"{item.get('speaker', '')}: {item.get('text', '')}")
            if recent_convs:
                context_parts.append("### Recent Conversation History:")
                context_parts.extend([f"- {c}" for c in recent_convs])

            return "\n".join(context_parts) if context_parts else "No relevant long-term memory records found."

    def consolidate_db(self) -> None:
        with self._lock:
            cutoff_dt = datetime.datetime.now() - datetime.timedelta(days=30)
            
            def prune(table):
                new_list = []
                for item in self.db[table]:
                    try:
                        ts = datetime.datetime.fromisoformat(item.get("timestamp", ""))
                        if ts < cutoff_dt and item.get("importance", 0.5) < 0.6 and item.get("access_count", 0) == 0:
                            continue # Prune
                    except:
                        pass
                    new_list.append(item)
                self.db[table] = new_list

            prune("experiences")
            prune("visual_memory")
            self._save_db()
            
        self.rebuild_faiss_index()
        logger.info("Memory database consolidation completed.")
        
    def close(self) -> None:
        # Save one last time
        self._save_db()
