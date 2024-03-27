import json

# use json to analyse data
def Json_transformer(data):
    
    parsed_data = {
        "header": data[:16],
        "text_info": data[16:44],
        "scan_data": [data[i:i+4] for i in range(44, len(data)-26, 8)],
        "re_data": [data[i+4:i+8] for i in range(44, len(data)-26, 8)],
        "timestamp": data[-22:-2],
        "checksum": data[-2:]
    }
    print(parsed_data['header'])
    range_list = list(map(lambda x: int(x, 16)/1e3, parsed_data["scan_data"]))
    re_list = list(map(lambda x: int(x, 16)/1.0, parsed_data["re_data"]))

    return parsed_data, [range_list, re_list]