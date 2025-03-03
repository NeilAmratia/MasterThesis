from llama_index.core import VectorStoreIndex, Document, Settings
from llama_index.llms.groq import Groq
from llama_index.core.node_parser import SimpleNodeParser
from llama_index.embeddings.huggingface import HuggingFaceEmbedding
from llama_index.vector_stores.faiss import FaissVectorStore
from typing import List
import os
import markdown
from bs4 import BeautifulSoup
import streamlit as st
from faiss import IndexFlatL2

class DocumentQA:
    def __init__(self, embedding_model: str = "sentence-transformers/all-MiniLM-L6-v2"):
        
        system_prompt = """You are a testing engineer identifying specific 2-way feature interactions in BCS.
        
        Available Features:
        1. Power Window Features:
           - ManualPowerWindow (MPW)
           - AutomaticPowerWindow (APW)
           - FingerProtection (FP)
        
        2. Mirror Features:
           - Electric (EM)
           - Heatable (HM)
        
        3. Security Features:
           - AlarmSystem (AS)
           - InteriorMonitoring (IM)
           - CentralLockingSystem (CLS)
           - AutomaticLocking (AL)
           - RemoteControlKey (RCK)
        
        4. LED Features:
           - LEDAlarmSystem (LAS)
           - LEDFingerProtection (LFP)
           - LEDCentralLockingSystem (LCLS)
           - LEDPowerWindow (LPW)
           - LEDExteriorMirror (LEM)
           - LEDHeatable (LH)
        
        Output Format:
        {Feature1, Feature2}: Brief reason for interaction testing
        
        Rules:
        - Use feature abbreviations in pairs
        - Only include interactions that have functional dependencies
        - Exclude purely visual/indicator relationships
        - Focus on safety and control dependencies
        """

        # Initialize Groq
        self.llm = Groq(
            model="mixtral-8x7b-32768",
            api_key="", # Enter key here
            temperature=0.1,
            max_tokens=1024,
            context_window=8076,
            top_p=0.1,
            streaming=False,
            system_prompt=system_prompt
        )
        
        # Initialize embeddings
        self.embed_model = HuggingFaceEmbedding(
            model_name=embedding_model
        )
        
        # Set global settings
        Settings.llm = self.llm
        Settings.embed_model = self.embed_model
        
        self.index = None
        self.conversation_history = []
        self.CHUNK_SIZE = 500

    def process_documents(self, file_paths: List[str]):
        """Process documents from file paths"""
        documents = []
        
        for file_path in file_paths:
            try:

                if file_path.endswith('.pdf'):
                    with open(file_path, 'rb') as file:
                        pdf_content = file.read()
                        markdown_text = self.pdf_to_markdown(pdf_content)
                        
                        # Validate content
                        if not markdown_text.strip():
                            print(f"Warning: Empty content in {file_path}")
                            continue
                            
                        doc = Document(
                            text=markdown_text,
                            metadata={
                                "file_path": file_path,
                                "file_type": "pdf",
                                "format": "markdown"
                            }
                        )
                        documents.append(doc)
                        
                elif file_path.endswith('.png'):
                    from llama_index.readers.file import ImageReader
                    loader = ImageReader()
                    docs = loader.load_data(file_path)
                    documents.extend(docs)

                elif file_path.endswith('.txt'):
                    with open(file_path, 'r', encoding='utf-8') as file:
                        content = file.read()
                        if not content.strip():
                            print(f"Warning: Empty content in {file_path}")
                            continue
                        
                        doc = Document(
                            text=content,
                            metadata={
                                "file_path": file_path,
                                "file_type": "text",
                                "format": "plain"
                            }
                        )
                        documents.append(doc)
                    
                elif file_path.endswith('.uvl'):
                    with open(file_path, 'r') as file:
                        content = file.read()
                        if not content.strip():
                            print(f"Warning: Empty content in {file_path}")
                            continue
                        doc = Document(text=content)
                        documents.append(doc)

            except Exception as e:
                print(f"Error processing {file_path}: {str(e)}")
                continue
        
        # Validate documents before processing
        if not documents:
            raise ValueError("No valid documents to process")

        try:
            parser = SimpleNodeParser.from_defaults(
                chunk_size=500,
                chunk_overlap=50
            )
            nodes = parser.get_nodes_from_documents(documents)
            
            # Validate nodes
            valid_nodes = [node for node in nodes if node.text.strip()]
            if not valid_nodes:
                raise ValueError("No valid content found in documents")
                
            faiss_index = IndexFlatL2(384)
            vector_store = FaissVectorStore(faiss_index=faiss_index)
            self.index = VectorStoreIndex.from_documents(
                documents,
                vector_store=vector_store
            )
        
        except Exception as e:
            print(f"Error during document processing: {str(e)}")
            raise

    def pdf_to_markdown(self, pdf_content):
        """Convert PDF content to markdown format"""
        from PyPDF2 import PdfReader
        from io import BytesIO
        
        markdown_text = []
        reader = PdfReader(BytesIO(pdf_content))
        
        for page in reader.pages:
            text = page.extract_text()
            
            if '|' in text:
                lines = text.split('\n')
                table_lines = []
                for line in lines:
                    if '|' in line:
                        cells = line.split('|')
                        table_lines.append('| ' + ' | '.join(cells) + ' |')
                if table_lines:
                    table_lines.insert(1, '|---' * (len(table_lines[0].split('|')) - 1) + '|')
                    text = '\n'.join(table_lines)
            
            lines = text.split('\n')
            for i, line in enumerate(lines):
                if line.strip():
                    if len(line.strip()) < 50 and line.strip().isupper():
                        lines[i] = f"## {line}"
            
            markdown_text.append('\n'.join(lines))
        
        return '\n\n'.join(markdown_text)

    def chat_with_llm(self, query: str) -> str:

        if not self.index:
            raise ValueError("Please process documents first")

        formatted_query = """List only the required 2-way feature interactions for testing in this format:
        {Feature1, Feature2}: Brief reason
        
        Focus only on interactions with:
        - Direct functional dependencies
        - Safety implications
        - Control relationships
        
        Do not include:
        - General explanations
        - LED indicator relationships
        - Abstract feature groups
        """
        
        query_engine = self.index.as_query_engine(
            similarity_top_k=3,
            response_mode="compact"
        )
        
        try:
            response = query_engine.query(formatted_query)
            return str(response)
        except Exception as e:
            if "rate_limit_exceeded" in str(e):
                return "Rate limit exceeded. Please wait a moment and try again with a smaller query."
            raise e

def display_chat_message(role: str, message: str):
    """Display a chat message with custom styling."""
    if role == "User":
        st.markdown(
            f'<div style="background-color: #f9f9f9; padding: 10px; border-radius: 5px; margin: 5px 0;">'
            f'<strong>{role}:</strong> {message}</div>',
            unsafe_allow_html=True
        )
    else:
        st.markdown(
            f"""
            <div style="background-color: {'#1e1e1e' if role == 'User' else '#2d2d2d'}; 
                        color: #ffffff;
                        padding: 15px; 
                        border-radius: 10px; 
                        margin: 10px 0; 
                        border: 1px solid {'#3d3d3d' if role == 'User' else '#4a4a4a'};">
                <strong style="color: {'#bb86fc' if role == 'Assistant' else '#03dac6'};">
                    {role}:
                </strong>
                <span style="color: #ffffff;">
                    {message}
                </span>
            </div>
            """,
            unsafe_allow_html=True
        )

def main():
    st.title("Code Analysis System")
    
    # Initialize session state
    if 'qa_system' not in st.session_state:
        st.session_state.qa_system = DocumentQA()
        
        # Process documents
        document_dir = "llm_input"
        file_paths = []
        for file in os.listdir(document_dir):
            if file.endswith(('.pdf', '.png', '.uvl', '.txt')):
                file_paths.append(os.path.join(document_dir, file))
        
        if file_paths:
            with st.spinner("Processing documents..."):
                st.session_state.qa_system.process_documents(file_paths)
                st.success("Documents processed successfully!")

    if 'chat_history' not in st.session_state:
        st.session_state.chat_history = []
    
    # Chat interface in a container
    chat_container = st.container()
    
    # Input area at the bottom
    with st.form(key='chat_form', clear_on_submit=True):
        query = st.text_area(
            "Ask about the documents:",
            key='chat_input',
            height=100,
            help="Enter your query here. Press Ctrl+Enter to submit."
        )
        submit_button = st.form_submit_button("Send")
        
        if submit_button and query:
            response = st.session_state.qa_system.chat_with_llm(query)
            st.session_state.chat_history.append(("User", query))
            st.session_state.chat_history.append(("Assistant", response))
            
    # Display chat history in the container
    with chat_container:
        for role, message in reversed(st.session_state.chat_history):
            display_chat_message(role, message)

def display_chat_message(role: str, message: str):
    """Display a chat message with improved styling."""
    if role == "User":
        bgcolor = "#0e1117"
        border_color = "#1e1e1e"
    else:
        bgcolor = "#1e1e1e"
        border_color = "#2e2e2e"
        
    st.markdown(
        f"""
        <div style="
            background-color: {bgcolor};
            border: 1px solid {border_color};
            border-radius: 10px;
            padding: 10px;
            margin: 5px 0;
        ">
            <strong style="color: {'#03dac6' if role == 'User' else '#bb86fc'};">
                {role}:
            </strong>
            <span style="color: #ffffff;">
                {message}
            </span>
        </div>
        """,
        unsafe_allow_html=True
    )

if __name__ == "__main__":
    main()