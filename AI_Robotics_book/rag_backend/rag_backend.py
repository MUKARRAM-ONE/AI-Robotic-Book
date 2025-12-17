import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
import tiktoken
import hashlib

class RAGBackend:
    def __init__(self):
        load_dotenv()
        qdrant_url = os.getenv("QDRANT_URL", ":memory:")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        
        # Handle both cloud and local (in-memory) Qdrant
        if qdrant_url == ":memory:":
            self.qdrant_client = QdrantClient(":memory:")
        else:
            self.qdrant_client = QdrantClient(
                url=qdrant_url, 
                api_key=qdrant_api_key if qdrant_api_key else None
            )
        
        self.collection_name = os.getenv("QDRANT_COLLECTION", "ai_robotics_docs")
        self.top_k = int(os.getenv("TOP_K", "4"))
        self.tokenizer = tiktoken.get_encoding("cl100k_base")
        
        # Initialize with sample data if using in-memory
        if qdrant_url == ":memory:":
            self._initialize_sample_data()

    def _initialize_sample_data(self):
        """Initialize with sample ROS 2 documentation for testing."""
        try:
            # Create collection
            self.qdrant_client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE)
            )
            
            # Sample ROS 2 documentation
            sample_docs = [
                {
                    "text": "ROS 2 (Robot Operating System 2) is a middleware for writing robot software. It provides services for hardware abstraction, device drivers, message passing between processes, and package management.",
                    "source": "01-intro-to-ros2.md",
                    "section": "What is ROS"
                },
                {
                    "text": "ROS 2 nodes are independent executable processes that perform computation. Nodes communicate with each other using topics (publish-subscribe), services (request-reply), and actions (asynchronous).",
                    "source": "02-ros2-nodes-topics-services.md",
                    "section": "ROS 2 Nodes"
                },
                {
                    "text": "Topics in ROS 2 enable anonymous publish-subscribe communication between nodes. Publishers send messages to topics and subscribers receive messages from topics. Topics are named data channels.",
                    "source": "02-ros2-nodes-topics-services.md",
                    "section": "Topics"
                },
                {
                    "text": "Services in ROS 2 provide synchronous request-reply communication. A service server implements a service and waits for service calls. A service client sends a request and waits for a response.",
                    "source": "02-ros2-nodes-topics-services.md",
                    "section": "Services"
                },
                {
                    "text": "Python with rclpy is a popular choice for ROS 2 development. The rclpy client library allows you to write nodes, publishers, subscribers, services, and clients in Python with async support.",
                    "source": "03-python-rclpy-integration.md",
                    "section": "Python ROS 2"
                },
                {
                    "text": "URDF (Unified Robot Description Format) is an XML format to describe a robot. URDF files define the robot's joints, links, and collision geometry. They are essential for simulation and visualization.",
                    "source": "04-urdf-for-humanoids.md",
                    "section": "URDF Basics"
                },
                {
                    "text": "Gazebo is a physics simulator for robotics. It allows you to simulate robots and environments with realistic physics. Gazebo plugins enable sensor simulation and custom dynamics.",
                    "source": "05-gazebo-fundamentals.md",
                    "section": "Gazebo Introduction"
                },
                {
                    "text": "Isaac Sim is NVIDIA's robot simulation platform built on Omniverse. It provides photorealistic rendering, physics simulation, and AI capabilities for robot development and testing.",
                    "source": "08-isaac-sim-intro.md",
                    "section": "Isaac Sim Basics"
                }
            ]
            
            # Generate simple embeddings (using hash-based vectors for demo)
            points = []
            for idx, doc in enumerate(sample_docs):
                # Create a simple vector from the text (hash-based)
                vector = self._text_to_vector(doc["text"])
                
                point = models.PointStruct(
                    id=idx,
                    vector=vector,
                    payload=doc
                )
                points.append(point)
            
            # Upload points to collection
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            
            print(f"✅ Initialized RAG with {len(sample_docs)} sample documents")
        except Exception as e:
            print(f"⚠️ Error initializing sample data: {e}")

    def _text_to_vector(self, text: str, dim: int = 384) -> list:
        """Generate a simple vector from text using hash (for demo purposes)."""
        # Split text into words and create a simple sparse-like vector
        words = text.lower().split()
        vector = [0.0] * dim
        
        for i, word in enumerate(words):
            # Use hash to get reproducible values
            hash_val = int(hashlib.md5(word.encode()).hexdigest(), 16)
            idx = hash_val % dim
            vector[idx] += 1.0 / (len(words) + 1)
        
        # Normalize
        norm = sum(v**2 for v in vector) ** 0.5
        if norm > 0:
            vector = [v / norm for v in vector]
        
        return vector

    def search_and_generate(self, query: str, top_k: int = None) -> dict:
        """
        Searches for a query in the vector database and returns the truncated context.
        """
        if top_k is None:
            top_k = self.top_k
            
        print(f"RAG Backend: Processing query: '{query}'")
        
        retrieved_docs = self._retrieve_context(query, top_k)
        
        print(f"RAG Backend: Retrieved {len(retrieved_docs)} documents.")
        
        if not retrieved_docs:
            return {
                "answer": "I couldn't find any relevant information in the documentation. Please try rephrasing your question.",
                "sources": []
            }
        
        # Combine the text from the retrieved documents
        context = " ".join([doc['text'] for doc in retrieved_docs])
        
        # Truncate to 100 tokens
        tokens = self.tokenizer.encode(context)
        if len(tokens) > 100:
            tokens = tokens[:100]
        
        answer = self.tokenizer.decode(tokens)
        
        return {"answer": answer, "sources": retrieved_docs}

    def _retrieve_context(self, query: str, top_k: int) -> list[dict]:
        """
        Retrieves context from the vector database.
        """
        try:
            query_vector = self._text_to_vector(query)
            
            search_result = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k
            )
            
            retrieved_docs = [hit.payload for hit in search_result]
            return retrieved_docs
        except Exception as e:
            print(f"Error retrieving context: {e}")
            import traceback
            traceback.print_exc()
            return []

if __name__ == '__main__':
    # This is an example of how to use the RAGBackend class.
    # It assumes that you have a Qdrant instance running and that the
    # 'ai_robotics_docs' collection has been populated with embeddings.
    
    rag_backend = RAGBackend()
    
    # Example query
    example_query = "What are ROS 2 nodes?"
    
    print(f"--- Running test query: '{example_query}' ---")
    response = rag_backend.search_and_generate(example_query)
    print("\n--- Response ---")
    print(response["answer"])
    print("\n--- End of test ---")
