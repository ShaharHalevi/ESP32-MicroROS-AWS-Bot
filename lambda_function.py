import json
import boto3
from decimal import Decimal

# Initialize services (outside the handler to save execution time/cold start)
dynamodb = boto3.resource('dynamodb')
table = dynamodb.Table('Storage_Packages') # Ensure this matches your exact DynamoDB table name
sns = boto3.client('sns')

# *** PASTE YOUR SNS TOPIC ARN HERE ***
SNS_TOPIC_ARN = 'XXXX' 

def lambda_handler(event, context):
    try:
        print("Received event:", json.dumps(event))
        
        # 1. Extract data from the incoming robot message
        # (Assumption: The IoT Rule passes clean JSON with box_id, food, count)
        box_id = event.get('box_id')
        timestamp = str(event.get('timestamp')) # DynamoDB requires timestamp as a String
        food = event.get('food')
        count = int(event.get('count'))
        
        # 2. Save data to DynamoDB (for Grafana visualization)
        table.put_item(Item={
            'box_id': box_id,
            'timestamp': timestamp,
            'food': food,
            'count': count
        })
        print(f"Data saved to DynamoDB: {food} count {count}")

        # 3. The "Brain" - Business Logic
        # If stock is below 20 units -> Send emergency alert!
        if count < 20:
            message = f"ALERT: Low stock detected!\nItem: {food}\nBox ID: {box_id}\nQuantity Remaining: {count}"
            
            sns.publish(
                TopicArn=SNS_TOPIC_ARN,
                Message=message,
                Subject='Robot Inventory Alert'
            )
            print("Alert sent to SNS")

        return {
            'statusCode': 200,
            'body': json.dumps('Process Completed Successfully')
        }

    except Exception as e:
        print(f"Error: {str(e)}")
        return {
            'statusCode': 500,
            'body': json.dumps(f"Error: {str(e)}")
        }
