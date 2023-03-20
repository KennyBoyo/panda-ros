import httpx
import json
import sys

import rospy

import hashlib

def encrypt_string(hash_string):
    sha_signature = \
        hashlib.sha256(hash_string.encode()).hexdigest()
    return sha_signature

password = 'uq-mrn-amclab'

encrypted_password = encrypt_string(password)

print(encrypted_password)

USERNAME = "amc-lab2"
ENCODED_PASSWORD = encrypted_password

rospy.init_node("unlock_arm")

ip = rospy.get_param("~robot_ip", "172.16.0.3")

client = httpx.Client(verify=False, base_url=f"https://172.16.0.2")


client.headers.update({
    "accept": "text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9",
    "accept-language": "en-US,en;q=0.9",
    "sec-ch-ua": "\" Not;A Brand\";v=\"99\", \"Google Chrome\";v=\"97\", \"Chromium\";v=\"97\"",
    "sec-ch-ua-mobile": "?0",
    "sec-ch-ua-platform": "\"Linux\"",
    "upgrade-insecure-requests": "1"
})

res = client.get(url="/admin/login")
res = client.get(url="/admin/api/first-start", headers={"content-type": "application/json"})
res = client.get(url="/admin/api/startup-phase", headers={"content-type": "application/json"})

login_body = {"login": USERNAME,
        "password": ENCODED_PASSWORD}

res = client.post(url="/admin/api/login", headers={"content-type": "application/json"}, content=json.dumps(login_body), )

if res.status_code == 200:
    print("Successfully logged in to robot")
else:
    print("Logging in to Franka Desk failed. Exiting.")
    sys.exit(1)


auth_cookie = res.text
client.headers.update({'Authorization': auth_cookie})

res = client.get(url="/admin/api/safety", headers={"content-type": "application/json"})
res = client.post(url="/admin/api/control-token/request", headers={"content-type": "application/json"}, json={"requestedBy": USERNAME})
auth_token = res.json()['token']

auth_token = "VJ9BFhG+TBvwXPAte1Ev0K4WerM3ECll0Y1cQeiLjdQ="

client.headers.update({'X-Control-Token': auth_token})

res = client.get(url="/admin/api/robot/shutdown-position-error")
res = client.post(url="/desk/api/robot/open-brakes", data={"force": "false"}, timeout=20)

if res.status_code == 200:
    print("Successfully deactivated robot hand lock")
else:
    print("Failed to deactivate robot hand lock")
    sys.exit(2)

res = client.post(url="/admin/api/control-token/fci", json={'token': auth_token})

if res.status_code == 200:
    print("Successfully activated fci")
else:
    print("Failed to activate fci")
    sys.exit(3)