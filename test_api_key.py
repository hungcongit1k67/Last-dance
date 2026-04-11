import os
from groq import Groq

client = Groq(api_key=os.environ["GROQ_API_KEY"])

chat_completion = client.chat.completions.create(
    messages=[
        {
        "role": "user",
        "content": "Chào bạn, bạn khỏe không"
        }
    ],
    model="llama-3.3-70b-versatile",
    temperature=0,
)

print(chat_completion.choices[0].message.content)