import requests
import json

class MiR_API:
    def __init__(self):
        self.ip = '172.31.1.148'
        self.host = f"http://{self.ip}/api/v2.0.0/"
        self.headers = {}
        self.headers['Content-Type'] = 'application/json'
        self.headers['Authorization'] = 'Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
        self.headers['Accept-Language'] = 'en_US'

    def get_state(self):
        url = 'status'

        try:
            url = self.host + url
            response = requests.get(url, self.headers)

            if response.status_code == 200:
                posts = response.json()
                return posts
            else:
                print('Error:', response.status_code)
                return None
        except requests.exceptions.RequestException as e:
            print('Error:', e)
            return None

    def get_missions(self):
        url = 'missions'

        try:
            url = self.host + url
            #print(url)
            response = requests.get(url, headers=self.headers)
            print(response.status_code)
            if response.status_code == 200:
                posts = response.json()
                return posts
            else:
                print('Error:', response.status_code)
                return None
        except requests.exceptions.RequestException as e:
            print('Error:', e)
            return None
        
    def get_registers(self):
        pass
    
    def post_append_mission(self):
        url = 'mission_queue'
        mission = {'mission_id': '65a06dcb-2448-11ef-80fe-000129af97ab'}
        try:
            url = self.host + url
            response = requests.post(url, json=mission, headers=self.headers)
            posts = response.json()
            if response.status_code == 201:
                posts = response.json()
                return posts
            else:
                print('Error:', response.status_code)
                return None

        except requests.exceptions.RequestException as e:
            print('Error:', e)
            return None

