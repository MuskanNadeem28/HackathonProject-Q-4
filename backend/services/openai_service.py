import openai
from typing import Optional
from config.settings import settings
import logging

logger = logging.getLogger(__name__)

class OpenAIService:
    def __init__(self):
        # Set the OpenAI API key
        openai.api_key = settings.OPENAI_API_KEY
        openai.base_url = settings.OPENAI_BASE_URL or "https://api.openai.com/v1"

        # Set default model
        self.default_model = settings.OPENAI_MODEL or "gpt-3.5-turbo"

    async def generate_response(
        self,
        query: str,
        context: str,
        max_tokens: int = 500,
        temperature: float = 0.7
    ) -> str:
        """
        Generate a response using OpenAI's API based on the query and context
        """
        try:
            # Create the prompt with context
            system_message = f"""
            You are an expert assistant for the Physical AI & Humanoid Robotics book.
            Answer questions based ONLY on the provided context from the book.
            If the context doesn't contain enough information to answer the question,
            clearly state that the information is not available in the provided context.

            Context:
            {context}
            """

            response = openai.chat.completions.create(
                model=self.default_model,
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": query}
                ],
                max_tokens=max_tokens,
                temperature=temperature,
                timeout=30  # 30 second timeout
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            logger.error(f"Error generating response from OpenAI: {str(e)}")
            raise e

    async def validate_response_accuracy(self, query: str, response: str, context: str) -> bool:
        """
        Validate if the response is consistent with the provided context
        """
        try:
            validation_prompt = f"""
            You are an expert validator for the Physical AI & Humanoid Robotics book.
            Determine if the following response is consistent with the provided context.
            The response should only contain information that is present in the context.
            If the response contains information not found in the context, it's hallucinated.

            Context: {context}

            Query: {query}

            Response: {response}

            Answer with "VALID" if the response is consistent with the context, or "INVALID" if it contains hallucinated information.
            """

            validation_response = openai.chat.completions.create(
                model=self.default_model,
                messages=[
                    {"role": "user", "content": validation_prompt}
                ],
                max_tokens=20,
                temperature=0.0
            )

            result = validation_response.choices[0].message.content.strip().upper()
            return result == "VALID"

        except Exception as e:
            logger.error(f"Error validating response accuracy: {str(e)}")
            # If validation fails, assume it's valid to not block the response
            return True