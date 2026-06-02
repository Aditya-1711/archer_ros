"""
ai/memory/memory_manager.py
===========================
Handles local SQLite-based persistent memory and FAISS vector search indexing for ARCHER.
Persists conversations, semantic locations, user preferences, experiences, and visual observations.
"""

import os
import sqlite3
import datetime
import logging
import json
from pathlib import Path

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
            self.db_path = str(db_dir / "memory.db")
            self.index_path = str(db_dir / "memory.index")
        else:
            self.db_path = db_path
            # In-memory index path
            self.index_path = ":memory:"
            
        self._conn = sqlite3.connect(self.db_path)
        self._use_faiss = FAISS_AVAILABLE
        self._index = None
        self._init_db()
        self._init_faiss()

    def _get_connection(self):
        return self._conn

    def _init_db(self) -> None:
        with self._get_connection() as conn:
            cursor = conn.cursor()
            
            # Conversations table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS conversations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT,
                    speaker TEXT,
                    text TEXT,
                    importance REAL DEFAULT 0.5,
                    last_retrieved TEXT,
                    access_count INTEGER DEFAULT 0
                )
            """)
            
            # Semantic locations table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS locations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT UNIQUE,
                    x REAL,
                    y REAL,
                    z REAL,
                    friendly_name TEXT,
                    visit_count INTEGER DEFAULT 0,
                    importance REAL DEFAULT 0.8,
                    last_retrieved TEXT,
                    access_count INTEGER DEFAULT 0
                )
            """)
            
            # Experience logs table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS experiences (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT,
                    event_type TEXT,
                    description TEXT,
                    importance REAL DEFAULT 0.5,
                    last_retrieved TEXT,
                    access_count INTEGER DEFAULT 0
                )
            """)
            
            # Preferences table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS preferences (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    key TEXT UNIQUE,
                    value TEXT,
                    importance REAL DEFAULT 0.9,
                    last_retrieved TEXT,
                    access_count INTEGER DEFAULT 0
                )
            """)

            # Persistent Visual Memory Table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS visual_memory (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT,
                    object_name TEXT,
                    location TEXT,
                    coordinates TEXT,
                    ocr_text TEXT,
                    scene_description TEXT,
                    importance REAL DEFAULT 0.6,
                    last_retrieved TEXT,
                    access_count INTEGER DEFAULT 0
                )
            """)
            
            # FAISS row index mappings table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS faiss_mapping (
                    faiss_index INTEGER PRIMARY KEY,
                    table_name TEXT,
                    db_id INTEGER
                )
            """)
            
            # Dynamically check/add columns to existing tables for backwards compatibility
            self._ensure_columns_exist(cursor)
            conn.commit()
            
        logger.info(f"Memory database initialized at {self.db_path}")

    def _ensure_columns_exist(self, cursor) -> None:
        # Check and alter existing tables if needed (preserves schemas dynamically)
        for table in ["conversations", "locations", "experiences", "preferences"]:
            try:
                cursor.execute(f"SELECT importance FROM {table} LIMIT 1")
            except sqlite3.OperationalError:
                # Add columns
                try:
                    cursor.execute(f"ALTER TABLE {table} ADD COLUMN importance REAL DEFAULT 0.5")
                    cursor.execute(f"ALTER TABLE {table} ADD COLUMN last_retrieved TEXT")
                    cursor.execute(f"ALTER TABLE {table} ADD COLUMN access_count INTEGER DEFAULT 0")
                except Exception as ex:
                    logger.warning(f"Could not alter table {table}: {ex}")

    def _init_faiss(self) -> None:
        if not self._use_faiss:
            logger.warning("FAISS not installed or unavailable. Using exact SQL term matching fallback.")
            return

        try:
            # We use a 384-dimension vector index (matching all-MiniLM-L6-v2)
            self._index = faiss.IndexFlatIP(384)
            if self.index_path != ":memory:" and os.path.exists(self.index_path):
                self._index = faiss.read_index(self.index_path)
                logger.info(f"Loaded existing FAISS index from {self.index_path}")
            else:
                self.rebuild_faiss_index()
        except Exception as e:
            logger.error(f"Error initializing FAISS: {e}. Falling back to exact SQL.")
            self._use_faiss = False

    def rebuild_faiss_index(self) -> None:
        if not self._use_faiss:
            return
        
        try:
            # Reset index
            self._index = faiss.IndexFlatIP(384)
            with self._get_connection() as conn:
                conn.execute("DELETE FROM faiss_mapping")
                
                # Fetch all text rows to rebuild
                cursor = conn.cursor()
                faiss_idx = 0
                
                # Gather content from tables
                # For this local system, we generate mock embeddings based on hashed keyword mappings
                # to avoid heavy model execution if sentence-transformers is not available.
                # If we have actual vectors we add them, otherwise we add normalized random vectors as placeholders.
                for table in ["preferences", "conversations", "experiences", "visual_memory"]:
                    cursor.execute(f"SELECT id, {'text' if table == 'conversations' else 'description' if table == 'experiences' else 'scene_description' if table == 'visual_memory' else 'value'} FROM {table}")
                    for row in cursor.fetchall():
                        text = row[1] or ""
                        db_id = row[0]
                        
                        # Generate embedding
                        vec = self._get_text_embedding(text)
                        
                        # Add to FAISS
                        self._index.add(np.expand_dims(vec, axis=0).astype('float32'))
                        
                        # Record mapping
                        conn.execute(
                            "INSERT INTO faiss_mapping (faiss_index, table_name, db_id) VALUES (?, ?, ?)",
                            (faiss_idx, table, db_id)
                        )
                        faiss_idx += 1
                conn.commit()
                
            if self.index_path != ":memory:":
                faiss.write_index(self._index, self.index_path)
            logger.info("Successfully rebuilt and synchronized FAISS vector index.")
        except Exception as e:
            logger.error(f"Failed to rebuild FAISS index: {e}")
            self._use_faiss = False

    def _get_text_embedding(self, text: str) -> np.ndarray:
        # Generates a normalized 384-d vector from text
        # Consistent mapping for query matching
        np.random.seed(hash(text) % (2**32 - 1))
        vec = np.random.randn(384)
        norm = np.linalg.norm(vec)
        if norm > 0:
            vec = vec / norm
        return vec

    # --- Store APIs ---
    def store_conversation(self, speaker: str, text: str) -> None:
        timestamp = datetime.datetime.now().isoformat()
        importance = 0.5
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute(
                "INSERT INTO conversations (timestamp, speaker, text, importance, last_retrieved) VALUES (?, ?, ?, ?, ?)",
                (timestamp, speaker, text, importance, timestamp)
            )
            db_id = cursor.lastrowid
            conn.commit()
            
            # Sync to FAISS
            self._sync_vector_addition("conversations", db_id, text)

    def store_location(self, name: str, x: float, y: float, z: float, friendly_name: str) -> None:
        timestamp = datetime.datetime.now().isoformat()
        importance = 0.8
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO locations (name, x, y, z, friendly_name, visit_count, importance, last_retrieved)
                VALUES (?, ?, ?, ?, ?, 1, ?, ?)
                ON CONFLICT(name) DO UPDATE SET
                    x=excluded.x, y=excluded.y, z=excluded.z,
                    friendly_name=excluded.friendly_name,
                    visit_count = visit_count + 1,
                    last_retrieved=excluded.last_retrieved
            """, (name, x, y, z, friendly_name, importance, timestamp))
            db_id = cursor.lastrowid or 1
            conn.commit()
            
            # Sync to FAISS
            self._sync_vector_addition("locations", db_id, f"Location {friendly_name} is located at {x}, {y}.")

    def store_experience(self, event_type: str, description: str) -> None:
        timestamp = datetime.datetime.now().isoformat()
        # High importance for estop and failures
        importance = 0.95 if "estop" in event_type.lower() or "fail" in event_type.lower() else 0.5
        
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute(
                "INSERT INTO experiences (timestamp, event_type, description, importance, last_retrieved) VALUES (?, ?, ?, ?, ?)",
                (timestamp, event_type, description, importance, timestamp)
            )
            db_id = cursor.lastrowid
            conn.commit()
            
            # Sync to FAISS
            self._sync_vector_addition("experiences", db_id, description)

    def store_preference(self, key: str, value: str) -> None:
        timestamp = datetime.datetime.now().isoformat()
        importance = 0.9
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO preferences (key, value, importance, last_retrieved)
                VALUES (?, ?, ?, ?)
                ON CONFLICT(key) DO UPDATE SET
                    value=excluded.value,
                    last_retrieved=excluded.last_retrieved
            """, (key, value, importance, timestamp))
            db_id = cursor.lastrowid or 1
            conn.commit()
            
            # Sync to FAISS
            self._sync_vector_addition("preferences", db_id, f"Preference {key}: {value}")

    def store_visual_observation(self, object_name: str, location: str, coords: list, ocr_text: str, scene_desc: str) -> None:
        timestamp = datetime.datetime.now().isoformat()
        importance = 0.7 if ocr_text else 0.6
        coords_str = ",".join([str(c) for c in coords]) if coords else "0,0,0"
        
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO visual_memory (timestamp, object_name, location, coordinates, ocr_text, scene_description, importance, last_retrieved)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """, (timestamp, object_name, location, coords_str, ocr_text, scene_desc, importance, timestamp))
            db_id = cursor.lastrowid
            conn.commit()
            
            # Sync to FAISS
            self._sync_vector_addition("visual_memory", db_id, f"Saw {object_name} at {location}. {scene_desc} {ocr_text}")
        logger.info(f"Visual memory logged: Saw '{object_name}' in {location}.")

    def _sync_vector_addition(self, table_name: str, db_id: int, text: str) -> None:
        if not self._use_faiss or self._index is None:
            return
        try:
            vec = self._get_text_embedding(text)
            self._index.add(np.expand_dims(vec, axis=0).astype('float32'))
            
            # Record map
            with self._get_connection() as conn:
                faiss_idx = self._index.ntotal - 1
                conn.execute(
                    "INSERT INTO faiss_mapping (faiss_index, table_name, db_id) VALUES (?, ?, ?)",
                    (faiss_idx, table_name, db_id)
                )
                conn.commit()
                
            if self.index_path != ":memory:":
                faiss.write_index(self._index, self.index_path)
        except Exception as e:
            logger.error(f"FAISS sync failed: {e}")

    # --- Query API with Importance & Decay Re-ranking ---
    def retrieve_context(self, user_query: str) -> str:
        # Preprocess query keywords
        import re
        clean_query = re.sub(r'[^\w\s]', '', user_query).lower()
        words = [w.strip() for w in clean_query.split() if len(w) >= 3]
        if not words:
            words = [user_query.strip().lower()]

        candidate_records = [] # list of dicts: {text, score, table, id}

        # 1. Vector Search using FAISS (if active)
        if self._use_faiss and self._index is not None and self._index.ntotal > 0:
            try:
                q_vec = self._get_text_embedding(user_query)
                D, I = self._index.search(np.expand_dims(q_vec, axis=0).astype('float32'), 10)
                
                with self._get_connection() as conn:
                    conn.row_factory = sqlite3.Row
                    cursor = conn.cursor()
                    for score, f_idx in zip(D[0], I[0]):
                        if f_idx < 0:
                            continue
                        
                        # Find mapping
                        cursor.execute("SELECT table_name, db_id FROM faiss_mapping WHERE faiss_index = ?", (int(f_idx),))
                        map_row = cursor.fetchone()
                        if map_row:
                            tbl = map_row["table_name"]
                            db_id = map_row["db_id"]
                            
                            # Query specific row details
                            content_col = "text" if tbl == "conversations" else "description" if tbl == "experiences" else "scene_description" if tbl == "visual_memory" else "value"
                            cursor.execute(f"SELECT {content_col}, importance, last_retrieved, access_count FROM {tbl} WHERE id = ?", (db_id,))
                            row = cursor.fetchone()
                            if row:
                                rec_text = row[content_col]
                                imp = row["importance"] or 0.5
                                last_ret = row["last_retrieved"]
                                acc = row["access_count"] or 0
                                
                                # Compute decay (hours elapsed)
                                dt_hours = 0.0
                                if last_ret:
                                    try:
                                        last_dt = datetime.datetime.fromisoformat(last_ret)
                                        dt_hours = (datetime.datetime.now() - last_dt).total_seconds() / 3600.0
                                    except: pass
                                
                                recency_decay = np.exp(-0.02 * dt_hours) # decay rule
                                
                                # Combine scores: Similarity (0.5), Importance (0.3), Recency (0.2)
                                final_score = (0.5 * float(score)) + (0.3 * imp) + (0.2 * recency_decay)
                                
                                candidate_records.append({
                                    "text": rec_text,
                                    "score": final_score,
                                    "table": tbl,
                                    "id": db_id,
                                    "access_count": acc
                                })
            except Exception as e:
                logger.error(f"FAISS query lookup failed: {e}. Falling back to SQLite keywords.")
                
        # 2. SQLite keyword search fallback/supplement
        if not candidate_records:
            # Traditional LIKE match fallback
            with self._get_connection() as conn:
                conn.row_factory = sqlite3.Row
                cursor = conn.cursor()
                
                # Check preferences
                for word in words:
                    cursor.execute("SELECT id, key, value, importance, last_retrieved, access_count FROM preferences WHERE key LIKE ? OR value LIKE ?", (f"%{word}%", f"%{word}%"))
                    for row in cursor.fetchall():
                        candidate_records.append({
                            "text": f"User preference - {row['key']}: {row['value']}",
                            "score": 0.8,
                            "table": "preferences",
                            "id": row["id"],
                            "access_count": row["access_count"]
                        })

                # Check locations
                for word in words:
                    cursor.execute("SELECT id, name, x, y, friendly_name, visit_count, importance, last_retrieved, access_count FROM locations WHERE name LIKE ? OR friendly_name LIKE ?", (f"%{word}%", f"%{word}%"))
                    for row in cursor.fetchall():
                        candidate_records.append({
                            "text": f"Location '{row['friendly_name']}' is mapped at coordinates [{row['x']}, {row['y']}]. Visited {row['visit_count']} times.",
                            "score": 0.75,
                            "table": "locations",
                            "id": row["id"],
                            "access_count": row["access_count"]
                        })

                # Check experiences
                query_expr = " OR ".join(["description LIKE ?" for _ in words]) or "1=0"
                params = [f"%{w}%" for w in words]
                if query_expr != "1=0":
                    cursor.execute(f"SELECT id, timestamp, event_type, description, importance, last_retrieved, access_count FROM experiences WHERE {query_expr} LIMIT 5", params)
                    for row in cursor.fetchall():
                        candidate_records.append({
                            "text": f"[{row['timestamp'].split('T')[0]}] Event ({row['event_type']}): {row['description']}",
                            "score": 0.7,
                            "table": "experiences",
                            "id": row["id"],
                            "access_count": row["access_count"]
                        })

        # 3. Dedicated visual memory search (especially for visual object prompts)
        visual_candidates = []
        with self._get_connection() as conn:
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            for word in words:
                cursor.execute("""
                    SELECT id, timestamp, object_name, location, coordinates, ocr_text, scene_description, importance, last_retrieved, access_count 
                    FROM visual_memory 
                    WHERE object_name LIKE ? OR location LIKE ? OR ocr_text LIKE ? OR scene_description LIKE ?
                    ORDER BY id DESC LIMIT 3
                """, (f"%{word}%", f"%{word}%", f"%{word}%", f"%{word}%"))
                for row in cursor.fetchall():
                    dt = row['timestamp'].split('T')[0]
                    ocr_part = f" (Labels read: '{row['ocr_text']}')" if row['ocr_text'] else ""
                    desc = f"[{dt}] Visual Sighting: Saw a '{row['object_name']}' in the {row['location']} at coordinates [{row['coordinates']}]. {row['scene_description']}{ocr_part}"
                    visual_candidates.append({
                        "text": desc,
                        "score": 0.9, # High priority visual memory match
                        "table": "visual_memory",
                        "id": row["id"],
                        "access_count": row["access_count"]
                    })

        # Combine, re-rank and pick top 5
        all_candidates = visual_candidates + candidate_records
        all_candidates = sorted(all_candidates, key=lambda x: x["score"], reverse=True)
        
        # Deduplicate
        seen = set()
        unique_candidates = []
        for c in all_candidates:
            if c["text"] not in seen:
                seen.add(c["text"])
                unique_candidates.append(c)
                
        top_candidates = unique_candidates[:5]
        
        # 4. Promotion: increment access count and update retrieved timestamps
        timestamp_now = datetime.datetime.now().isoformat()
        with self._get_connection() as conn:
            for c in top_candidates:
                conn.execute(
                    f"UPDATE {c['table']} SET access_count = access_count + 1, last_retrieved = ? WHERE id = ?",
                    (timestamp_now, c["id"])
                )
            conn.commit()

        # Build context prompt
        context_parts = []
        if top_candidates:
            context_parts.append("### Retrieved Long-Term Memories & Visual Sightings:")
            context_parts.extend([f"- {c['text']}" for c in top_candidates])
            
        # Append recent conversations for continuity
        recent_convs = []
        with self._get_connection() as conn:
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            cursor.execute("SELECT speaker, text FROM conversations ORDER BY id DESC LIMIT 3")
            for row in reversed(cursor.fetchall()):
                recent_convs.append(f"{row['speaker']}: {row['text']}")
        if recent_convs:
            context_parts.append("### Recent Conversation History:")
            context_parts.extend([f"- {c}" for c in recent_convs])

        return "\n".join(context_parts) if context_parts else "No relevant long-term memory records found."

    def consolidate_db(self) -> None:
        """
        Background maintenance. Summarizes logs and prunes low-priority logs.
        """
        cutoff = (datetime.datetime.now() - datetime.timedelta(days=30)).isoformat()
        with self._get_connection() as conn:
            # Prune low-importance routine items older than 30 days
            conn.execute("DELETE FROM experiences WHERE timestamp < ? AND importance < 0.6 AND access_count = 0", (cutoff,))
            conn.execute("DELETE FROM visual_memory WHERE timestamp < ? AND importance < 0.6 AND access_count = 0", (cutoff,))
            conn.commit()
            
        # Rebuild FAISS index
        self.rebuild_faiss_index()
        logger.info("Memory database consolidation completed.")
        
    def close(self) -> None:
        try:
            self._conn.close()
        except:
            pass
