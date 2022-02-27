import argparse
import os
import time
import googleapiclient.discovery
from six.moves import input
from pprint import pprint
from googleapiclient import discovery
from oauth2client.client import GoogleCredentials
import time
import requests
import uuid
import subprocess
import logging

class GCP:
        def __init__(self, project_id, project_number, gce_instance_type="f1-micro", disk_size=30,
                 network="default", region="us-west1", zone="us-west1-a", **kwargs):
        super().__init__(**kwargs)
        self.cloud_service_provider = "GCP"
        self.credentials = GoogleCredentials.get_application_default()
        self.compute = googleapiclient.discovery.build('compute', 'v1', credentials=self.credentials)
        self.oslogin = googleapiclient.discovery.build('oslogin', 'v1')
        self.project_id = project_id
        self.network = network
        self.zone = zone
        self.disk_image = None
        self.project_number = project_number

        self.region = region
        self.gce_instance_type = "zones/" + self.zone + "/machineTypes/" + gce_instance_type
        self.instance_name = self.project_id + self.unique_name
        self.create()

    def create(self):
        self.create_gce_instance()
        self.connect()
        self.install_ros()
        self.install_colcon()
        self.install_cloud_dependencies()
        self.push_ros_workspace()
        #self.push_to_cloud_nodes()
        self.info(flush_to_disk = True)
        self.set_ready_state()

    def create_gce_instance(self):
        # choose image
        image_response = self.compute.images().getFromFamily(project='debian-cloud', family='debian-9').execute()
        self.disk_image = image_response['selfLink']

        # configure machine

        # startup_script = open(os.path.join(os.path.dirname(__file__), 'startup-script.sh'), 'r').read()

        config = {
            'name': self.instance_name,
            'machineType': self.gce_instance_type,

            'disks': [
                {
                    'boot': True,
                    'autoDelete': True,
                    'initializeParams': {
                        'sourceImage': self.disk_image,
                    }
                }
            ],

            'networkInterfaces': [{
                'network': 'global/networks/default',
                'accessConfigs': [
                    {'type': 'ONE_TO_ONE_NAT', 'name': 'External NAT'}
                ]
            }],

            'serviceAccounts': [{
                'email': 'default',
                'scopes': [
                    'https://www.googleapis.com/auth/devstorage.read_write',
                    'https://www.googleapis.com/auth/logging.write'
                ]
            }],

            'metadata': {
                # 'items': [{
                #    'key': 'startup-script',
                #    'value': startup_script
                # }]
            }
        }
        ret=  self.compute.instances().insert(project=self.project_number, zone=self.zone, body=config).execute()
        import time
        time.sleep(10)
        return ret

    def delete(self):
        return self.compute.instances().delete(project=self.project_number, zone=self.zone,
                                               instance=self.project_id).execute()

    def create_security_group(self):
        network_body = {
            "rountingConfig": {
                "routingMode": "REGIONAL"
            },
            "autoCreateSubnetworks": False,
            "name": self.network,
            "mtu": 1460,
            "region": self.region
        }

        subnetwork_body = {
            "enableFlowLogs": False,
            "ipCidrRange": "0.0.0.0/0",
            "name": "snet",
            "network": "projects/" + self.project_id + "global/networks" + self.network,
            "privateIpGoogleAccess": False,
            "region": self.region
        }

        request1 = self.compute.networks().insert(project=self.project_number, region=self.region, body=network_body)
        response1 = request1.execute()

        pprint(response1)

        print("creating network . . .\n")
        time.sleep(30)

        request2 = self.compute.subnetworks().insert(project=self.project_number, region=self.region, body=subnetwork_body)
        response2 = request2.execute()

        pprint(response2)


    def get_ip(self):
        response = self.compute.instances().get(project=self.project_number, zone=self.zone,
                                                instance=self.instance_name).execute()
        print(response)
        ip = response['networkInterfaces'][0]['accessConfigs'][0]['natIP']
        return ip

    def generate_key_pair(self):
        SERVICE_ACCOUNT_METADATA_URL = (
            'http://metadata.google.internal/computeMetadata/v1/instance/'
            'service-accounts/default/email')
        HEADERS = {'Metadata-Flavor': 'Google'}

        account = request.get(SERVICE_ACCOUNT_METADATA_URL, headers=HEADERS).text
        if not account.startswith('users/'):
            account = 'users/' + account
        
        private_key_file = '/tmp/key-' + str(uuid.uuid4())
        execute(['ssh-keygen', '-t', 'rsa', '-N', '', '-f', private_key_file])

        with open(private_key_file + '.pub', 'r') as original:
            public_key = original.read().strip()
        
        expiration = int((time.time() + 300) * 1000000)

        body = {
            'key': public_key,
            'expirationTimeUsec': expiration,
        }
        self.oslogin.users().importSshPublicKey(parent=account, body=body).execute()
        return private_key_file

    def exec_cmd(self, cmd):
        #profile = self.oslogin.users().getLoginProfile(name=account).execute()
        #username = profile.get('posixAccounts')[0].get('username')

        #hostname = '{instance}.{zone}.c.{project}.internal'.format(
        #    instance=self.instance_name, zone=self.zone, project=self.project_number)
        
        #could use get_ip for hostname
        ssh_command = [
            'ssh', '-i', private_key_file, '-o', 'StrictHostKeyChecking=no',
            "{username}@{hostname}".format(username=username, hostname=hostname), cmd
        ]

        ssh = subprocess.Popen(
            ssh_command, shell=False, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE)
        result = ssh.stdout,readlines()
        return result
