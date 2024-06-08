from openai import OpenAI
import os

client = OpenAI()
OpenAI.api_key = os.getenv('OPENAI_API_KEY')

# response = client.chat.completions.create(
#     model="gpt-3.5-turbo",
#     messages=[
#         {"role": "user", "content": "what is highest mountain in Japan."},
#     ],
#     temperature=0.2,
#     max_tokens=100
# )
# print(response)

functions = [
    {
        "name" : "get_current_wather",
        "description": "Get the currrent weather in a given location",
        "parameters" : {
            "type": "object",
            "properties": {
                "location": {
                    "type": "string",
                    "description": "The city and state, e.g. Tokyo, Yokohama",
                },
            },
        },
    },
]

my_messages=[
    {"role": "user", "content": "what is whather like in Tokyo."},
]


response = client.chat.completions.create(
    model="gpt-3.5-turbo",
    messages=my_messages,
    functions = functions,
    temperature=0.2,
    max_tokens=100
)
print(response)

print(response.choice[0].finish_reacon)
