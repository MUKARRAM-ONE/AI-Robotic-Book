# Physical AI & Humanoid Robotics Textbook

A comprehensive, AI-powered interactive textbook covering ROS 2, robotics simulation, digital twins, and vision-language models for physical AI applications.

## 📚 What's Inside

The textbook covers 4 major modules across 13 chapters:

### **Module 1: ROS 2 Fundamentals** (Chapters 1-3)
- Introduction to ROS 2 and its architecture
- Understanding nodes, topics, services, and actions
- Building applications with rclpy

### **Module 2: Digital Twin & Simulation** (Chapters 4-8)
- URDF and robot description formats
- Gazebo physics simulation
- Unity high-fidelity visualization
- Comprehensive sensor simulation
- NVIDIA Isaac Sim introduction

### **Module 3: AI-Robot Brain** (Chapters 9-10)
- Isaac ROS and VSLAM for perception
- Nav2 path planning and autonomous navigation

### **Module 4: Vision Language Agent** (Chapters 11-13)
- Voice-to-action with Whisper ASR
- Cognitive planning with Large Language Models
- Capstone project bringing it all together

## 🚀 Quick Start

### Prerequisites
- Node.js 16+ and npm
- Python 3.9+
- OpenAI API key (for RAG search)
- Qdrant instance (cloud or local)

### 1. Start the Textbook

```bash
cd docs
npm install
npm start
```

The textbook will open at `http://localhost:3000`

### 2. Set Up RAG Backend (Optional but Recommended)

The RAG backend provides AI-powered search across all documentation.

```bash
# Set up Python environment
cd rag_backend
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your OpenAI and Qdrant credentials

# Ingest documentation into vector database
python ingest.py

# Start the RAG API server
python server.py
```

The API will be available at `http://localhost:8001`

### 3. Connect Textbook to RAG Backend

When the RAG backend is running, the search component on the textbook homepage will automatically connect to it.

## 📖 Features

### Content
- **13 Comprehensive Chapters**: Deep dive into modern robotics development
- **Hands-On Code Examples**: Real Python and C++ code samples throughout
- **Visual Architecture Diagrams**: SVG diagrams explaining complex systems
- **Real-World Examples**: Warehouse robots, humanoid robots, VLMs, and more

### Interactive Features
- **AI-Powered Search**: Ask questions about robotics and get intelligent answers
- **Code Highlighting**: Syntax-highlighted code blocks for all major languages
- **Responsive Design**: Works on desktop, tablet, and mobile devices
- **Dark Mode Support**: Eye-friendly dark theme for late-night reading

### Technical Stack
- **Frontend**: Docusaurus 2 (React-based documentation site)
- **Search Backend**: FastAPI + LangChain + Qdrant + OpenAI
- **Visualization**: SVG architecture diagrams, image galleries
- **Responsive**: Mobile-friendly design with TailwindCSS styling

## 🏗️ Project Structure

```
AI_Robotics_book/
├── docs/                          # Docusaurus website
│   ├── src/
│   │   ├── components/           # React components (SearchBar, etc.)
│   │   ├── css/                  # Styling
│   │   └── pages/                # Landing page
│   ├── docs/                     # Markdown documentation
│   │   ├── 01-intro-to-ros2.md
│   │   ├── 02-ros2-nodes-topics-services.md
│   │   └── ... (13 chapters total)
│   ├── static/
│   │   └── img/                  # Images for all chapters
│   ├── docusaurus.config.js      # Docusaurus configuration
│   └── package.json
│
├── rag_backend/                  # RAG API backend
│   ├── rag_backend.py           # Main RAG implementation
│   ├── server.py                # FastAPI server
│   ├── ingest.py                # Document ingestion script
│   ├── requirements.txt          # Python dependencies
│   ├── .env.example              # Environment variables template
│   └── SETUP_GUIDE.md            # Detailed setup instructions
│
├── src/                          # Source code examples
│   ├── module1/                  # ROS 2 fundamentals
│   ├── module2/                  # Digital twin & simulation
│   ├── module3/                  # AI robot brain
│   └── module4/                  # Vision language agent
│
├── research/                     # Research papers and sources
├── specs/                        # Project specifications
└── README.md                     # This file
```

## 🔍 Search Features

The textbook includes an AI-powered search component that:

1. **Retrieves Relevant Content**: Uses vector embeddings to find relevant documentation sections
2. **Generates Answers**: Uses GPT-4 to synthesize coherent answers from retrieved content
3. **Shows Sources**: Displays which documentation sections were used to generate answers
4. **Real-Time**: Updates instantly as you type

### Example Queries
- "How do I create a ROS 2 publisher in Python?"
- "What are the differences between services and topics?"
- "How does SLAM work in robotics?"
- "How do I set up Isaac Sim?"
- "What is a vision language model?"

## 🛠️ Development

### Building the Textbook

```bash
cd docs
npm run build
```

This creates a static site in `docs/build/` that can be deployed anywhere.

### Updating Content

All documentation is in Markdown format in `docs/docs/`. Simply edit the files and the site will hot-reload.

### Adding Images

Place images in `docs/static/img/` and reference them with:
```markdown
![Alt text](/img/image-name.jpg)
```

### Extending the RAG Backend

The RAG backend is fully customizable:
- Change embedding models in `rag_backend.py`
- Adjust chunking strategy in `ingest.py`
- Add authentication in `server.py`
- Deploy to cloud services (Heroku, AWS, etc.)

## 📡 API Documentation

### RAG Backend Endpoints

#### POST `/query`
Send a natural language query and receive an intelligent answer.

**Request:**
```bash
curl -X POST "http://localhost:8001/query" \
  -H "Content-Type: application/json" \
  -d '{"query": "How do I create a ROS 2 node?"}'
```

**Response:**
```json
{
  "answer": "To create a ROS 2 node in Python...",
  "sources": [
    {
      "source": "03-python-rclpy-integration.md",
      "text": "Relevant content excerpt...",
      "score": 0.95
    }
  ]
}
```

## 🚢 Deployment

### Deploy Textbook to GitHub Pages

```bash
cd docs
npm run deploy
```

### Deploy RAG Backend

Options:
1. **Docker**:
   ```bash
   docker build -t rag-backend -f rag_backend/Dockerfile .
   docker run -p 8001:8001 rag-backend
   ```

2. **Heroku**:
   ```bash
   heroku create your-rag-backend
   git push heroku main
   ```

3. **Cloud Run (Google Cloud)**:
   ```bash
   gcloud run deploy rag-backend --source ./rag_backend
   ```

## 📊 Statistics

- **13 Chapters** covering all aspects of modern robotics
- **40+ Architecture Diagrams** explaining complex systems
- **Comprehensive Code Examples** in Python and C++
- **1000+ Lines** of well-documented content
- **AI-Powered Search** across all material

## 🤝 Contributing

To contribute to the textbook:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-content`)
3. Add your improvements or fixes
4. Submit a pull request

## 📝 License

This textbook and all code examples are provided under the MIT License. See `LICENSE` file for details.

## 🙏 Acknowledgments

- **ROS 2 Community**: For the incredible middleware
- **NVIDIA**: For Isaac Sim and Isaac ROS
- **OpenAI**: For GPT models and Whisper ASR
- **Qdrant**: For the vector database
- **Docusaurus Team**: For the documentation framework

## 📧 Contact & Support

For questions, suggestions, or bug reports:
- **GitHub Issues**: Open an issue in this repository
- **GitHub Discussions**: Join our community discussions
- **LinkedIn**: [Connect with the author](https://www.linkedin.com/in/mukarram-razzaq-0146572ba/)
- **GitHub**: [@MUKARRAM-ONE](https://github.com/MUKARRAM-ONE)

## 🔗 Useful Resources

- **ROS 2 Documentation**: https://docs.ros.org/
- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **Gazebo Docs**: https://gazebosim.org/docs
- **Isaac Sim Docs**: https://docs.omniverse.nvidia.com/isaacsim/latest/
- **Nav2 Documentation**: https://navigation.ros.org/
- **OpenAI API Docs**: https://platform.openai.com/docs/

## 🎯 Roadmap

Future enhancements:
- [ ] Interactive code editor with ROS 2 simulation
- [ ] Video tutorials for each chapter
- [ ] Community-contributed examples
- [ ] Multi-language support
- [ ] Offline-capable PWA version
- [ ] LaTeX math rendering in all chapters

## 📈 Version History

- **v1.0** (Current): Initial release with 13 chapters, RAG search, and comprehensive content

---

**Last Updated**: December 2024  
**Maintained By**: AI-driven Development Community