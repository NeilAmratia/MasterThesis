import streamlit as st
from langchain_community.document_loaders import PyPDFLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_community.embeddings import HuggingFaceEmbeddings
from langchain_community.vectorstores import FAISS
from openai import OpenAI
from pysat.solvers import Glucose3
from typing import List, Dict

class DocumentQA:
    def __init__(self, pdf_path: str, embedding_model: str = "sentence-transformers/all-MiniLM-L6-v2"):
        self.client = OpenAI()  # Enter key here
        self.embeddings = HuggingFaceEmbeddings(model_name=embedding_model)
        self.pdf_path = pdf_path
        self.vector_store = None
        self.conversation_history = []
        
    def process_pdf(self):
        loader = PyPDFLoader(self.pdf_path)
        pages = loader.load()
        
        splitter = RecursiveCharacterTextSplitter(
            chunk_size=500,
            chunk_overlap=50,
            length_function=len
        )
        
        chunks = splitter.split_documents(pages)
        self.vector_store = FAISS.from_documents(chunks, self.embeddings)
        
    def get_relevant_context(self, query: str, k: int = 3) -> str:
        if not self.vector_store:
            raise ValueError("Please process the PDF first")
        docs = self.vector_store.similarity_search(query, k=k)
        return "\n".join(doc.page_content for doc in docs)
    
    def chat_with_llm(self, query: str) -> str:
        context = self.get_relevant_context(query)
        
        # Include conversation history for context
        messages = [
            {"role": "system", "content": "You are a helpful assistant analyzing a document."}
        ]
        
        # Add conversation history
        for msg in self.conversation_history[-3:]:  # Keep last 3 exchanges for context
            messages.append(msg)
            
        # Add current query
        messages.append({
            "role": "user",
            "content": f"Based on this document context: {context}\n\nQuery: {query}"
        })
        
        response = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=messages,
            temperature=0.7
        )
        
        # Store the exchange in conversation history
        self.conversation_history.append({"role": "user", "content": query})
        self.conversation_history.append({"role": "assistant", "content": response.choices[0].message.content})
        
        return response.choices[0].message.content
    
    def generate_sat_problem(self) -> Dict:
        # Create a summary of the conversation for SAT problem generation
        conversation_summary = "\n".join([
            f"{'User' if msg['role'] == 'user' else 'Assistant'}: {msg['content']}"
            for msg in self.conversation_history[-6:]  # Last 6 messages
        ])
        
        # Ask LLM to generate SAT constraints based on conversation
        constraint_prompt = f"""Based on this conversation, generate SAT constraints and explain the problem:
        
        Conversation:
        {conversation_summary}
        
        Generate:
        1. A clear problem statement
        2. SAT constraints in CNF form (e.g., [[1, -2], [2, 3], [-1, -3]])
        3. Variable meanings (e.g., 1 means 'feature A is selected')
        """
        
        response = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a helpful assistant that generates SAT problems from conversations."},
                {"role": "user", "content": constraint_prompt}
            ],
            temperature=0.2
        )
        
        # Parse response and solve
        sat_response = response.choices[0].message.content
        
        # Find constraints in the response (simplified parsing)
        try:
            start_idx = sat_response.find("[[")
            end_idx = sat_response.find("]]") + 2
            constraints_text = sat_response[start_idx:end_idx]
            constraints = eval(constraints_text)
        except:
            constraints = [[1, 2], [-1]]  # Default fallback
        
        # Solve SAT problem
        solver = ConfigurationSolver()
        for clause in constraints:
            solver.add_constraint(clause)
        
        solution = solver.solve()
        
        return {
            "problem_description": sat_response,
            "constraints": constraints,
            "solution": solution
        }

class ConfigurationSolver:
    def __init__(self):
        self.solver = Glucose3()
        
    def add_constraint(self, clause: List[int]):
        self.solver.add_clause(clause)
        
    def solve(self) -> List[int]:
        if self.solver.solve():
            return self.solver.get_model()
        return None

# Streamlit interface
st.title("Document QA with SAT Solver")

# Initialize session state
if 'chat_history' not in st.session_state:
    st.session_state.chat_history = []

# PDF Processing
pdf_file = st.text_input("Enter PDF path:", "bcs_tubs_tech_rep_V1_4.pdf")

if 'qa_system' not in st.session_state and st.button("Process PDF"):
    with st.spinner("Processing PDF..."):
        st.session_state.qa_system = DocumentQA(pdf_file)
        st.session_state.qa_system.process_pdf()
        st.success("PDF processed successfully!")

if 'qa_system' in st.session_state:
    # Chat interface
    query = st.text_input("Ask about the document:")
    
    if query:
        response = st.session_state.qa_system.chat_with_llm(query)
        st.session_state.chat_history.append(("You", query))
        st.session_state.chat_history.append(("Assistant", response))
    
    # Display chat history
    st.subheader("Conversation History")
    for role, message in st.session_state.chat_history:
        st.write(f"**{role}:** {message}")
    
    # SAT Solver Integration
    if st.button("Generate and Solve SAT Problem from Conversation"):
        with st.spinner("Generating SAT problem..."):
            sat_result = st.session_state.qa_system.generate_sat_problem()
            
            st.subheader("SAT Problem Analysis")
            st.write("Problem Description:")
            st.write(sat_result["problem_description"])
            
            st.write("Generated Constraints:")
            st.write(sat_result["constraints"])
            
            st.write("Solution:")
            st.write(sat_result["solution"])