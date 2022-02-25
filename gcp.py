import argparse
import os
import time
import googleapiclient.discovery
from six.moves import input
from pprint import pprint
from googleapiclient import discovery
from oauth2client.client import GoogleCredentials
import time

class GCP:
    def __init__(self, region="us-west1", gce_instance_type="f1-micro", disk_size=30, project_id="exemplary-sign-342202", network="default", zone="us-west1-a"):
        self.credentials = GoogleCredentials.get_application_default()
        self.compute = googleapiclient.discovery.build('compute', 'v1', credentials=self.credentials)
        self.cloud_service_provider = "GCP"
        self.project_id = project_id
        self.network = network
        self.zone = zone
        self.disk_image = None
        self.project = "269519748100"

        self.region = region
        self.gce_instance_type = "zones/" + self.zone + "/machineTypes/" + gce_instance_type
    
    def create_gce_instance(self):
        # choose image
        image_response = self.compute.images().getFromFamily(project='debian-cloud', family='debian-9').execute()
        self.disk_image = image_response['selfLink']

        # configure machine
        
        #startup_script = open(os.path.join(os.path.dirname(__file__), 'startup-script.sh'), 'r').read()
        
        config = {
            'name': self.project_id,
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

            'networkInterfaces':[{
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
                #'items': [{
                #    'key': 'startup-script',
                #    'value': startup_script
                #}]
            }
        }
        return self.compute.instances().insert(project='269519748100', zone=self.zone, body=config).execute()

    def delete(self):
        return self.compute.instances().delete(project='269519748100', zone=self.zone, instance=self.project_id).execute()

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

        request1 = self.compute.networks().insert(project=self.project, region=self.region, body=network_body)
        response1 = request1.execute()

        pprint(response1)

        print("crreating network . . .\n")
        time.sleep(30)

        request2 = self.compute.subnetworks().insert(project=self.project, region=self.region, body=subnetwork_body)
        response2 = request2.execute()

        pprint(response2)
    
    def get_ip(self):
        response = self.compute.instances().get(project=self.project, zone=self.zone, instance=self.project_id).execute()
        ip = response['networkInterfaces'][0]['accessConfigs'][0]['natIP']
        return ip
