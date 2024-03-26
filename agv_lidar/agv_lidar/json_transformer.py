import json

def Json_transformer(data):
    
    # Parse the data into structured format
    parsed_data = {
        "header": data[:16],
        "text_info": data[16:44],
        "scan_data": [data[i:i+8] for i in range(44, len(data)-26, 8)],
        "timestamp": data[-22:-2],
        "checksum": data[-2:]
    }

    # Convert to JSON
    json_data = json.dumps(parsed_data, indent=4)
    return json_data