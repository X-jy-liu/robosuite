import openai
import requests
import os
from datetime import datetime

def check_openai_balance():
    """
    Check OpenAI API account balance and usage
    Returns True if balance is zero/insufficient, False if balance exists
    """
    
    # Get API key from environment variable
    api_key = os.getenv('OPENAI_API_KEY')
    
    if not api_key:
        print("Error: OPENAI_API_KEY environment variable not set")
        return None
    
    # Set API key for older openai library
    openai.api_key = api_key
    
    # Set up headers for API requests
    headers = {
        'Authorization': f'Bearer {api_key}',
        'Content-Type': 'application/json'
    }
    
    try:
        # Try to make a simple API call to test if account is active
        test_response = requests.get(
            "https://api.openai.com/v1/models",
            headers=headers,
            timeout=10
        )
        
        if test_response.status_code == 401:
            print("❌ Invalid API key")
            return None
        elif test_response.status_code == 429:
            print("❌ Rate limit exceeded or insufficient balance")
            return True
        elif test_response.status_code == 403:
            print("❌ Access forbidden - likely insufficient balance or account issue")
            return True
        elif test_response.status_code == 200:
            print("✅ API key is valid and account is accessible")
            
            # Try to make a minimal completion request using older API
            try:
                # Make a very small test request with older openai library
                response = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo",
                    messages=[{"role": "user", "content": "Hi"}],
                    max_tokens=1
                )
                print("✅ Successfully made test API call - account has sufficient balance")
                return False
                
            except openai.error.AuthenticationError:
                print("❌ Authentication failed - invalid API key")
                return None
            except openai.error.PermissionError:
                print("❌ Permission denied - check API key permissions")
                return None
            except openai.error.RateLimitError as e:
                print("❌ Rate limit or insufficient balance")
                if "insufficient" in str(e).lower() or "quota" in str(e).lower():
                    print("   Specifically: insufficient balance detected")
                return True
            except openai.error.APIError as e:
                if "insufficient" in str(e).lower() or "balance" in str(e).lower() or "quota" in str(e).lower():
                    print("❌ Insufficient balance detected")
                    return True
                else:
                    print(f"❌ API Error: {e}")
                    return None
            except openai.error.OpenAIError as e:
                if "insufficient" in str(e).lower() or "balance" in str(e).lower() or "quota" in str(e).lower():
                    print("❌ Insufficient balance detected")
                    return True
                else:
                    print(f"❌ OpenAI Error: {e}")
                    return None
            except Exception as e:
                error_str = str(e).lower()
                if "insufficient" in error_str or "balance" in error_str or "quota" in error_str:
                    print("❌ Insufficient balance detected")
                    return True
                else:
                    print(f"❌ Unexpected error during test call: {e}")
                    return None
        else:
            print(f"❌ Unexpected response code: {test_response.status_code}")
            print(f"Response: {test_response.text}")
            return None
            
    except requests.RequestException as e:
        print(f"❌ Network error: {e}")
        return None

def check_balance_alternative():
    """
    Alternative method using direct HTTP requests to check account status
    """
    api_key = os.getenv('OPENAI_API_KEY')
    
    if not api_key:
        print("Error: OPENAI_API_KEY environment variable not set")
        return None
    
    headers = {
        'Authorization': f'Bearer {api_key}',
    }
    
    try:
        # Try to access the subscription endpoint
        response = requests.get(
            "https://api.openai.com/v1/dashboard/billing/subscription",
            headers=headers,
            timeout=10
        )
        
        if response.status_code == 200:
            data = response.json()
            print("📊 Subscription info retrieved:")
            print(f"   Plan: {data.get('plan', {}).get('title', 'Unknown')}")
            
            # Check if there's a hard limit set
            hard_limit = data.get('hard_limit_usd')
            if hard_limit:
                print(f"   Hard limit: ${hard_limit}")
            
            return False  # If we can get subscription info, account is likely active
            
        elif response.status_code in [401, 403, 429]:
            print("❌ Cannot access billing information - likely insufficient balance or invalid key")
            return True
        else:
            print(f"⚠️  Unexpected response: {response.status_code}")
            return None
            
    except requests.RequestException as e:
        print(f"❌ Error checking subscription: {e}")
        return None

if __name__ == "__main__":
    print("🔍 Checking OpenAI API balance status...")
    print("=" * 50)
    
    # Method 1: Test API call
    print("\n📋 Method 1: Testing API accessibility...")
    no_balance = check_openai_balance()
    
    # Method 2: Check subscription (may require different permissions)
    print("\n📋 Method 2: Checking subscription info...")
    check_balance_alternative()
    
    print("\n" + "=" * 50)
    if no_balance is True:
        print("🚫 RESULT: No balance or insufficient funds detected")
    elif no_balance is False:
        print("💰 RESULT: Account appears to have sufficient balance")
    else:
        print("❓ RESULT: Unable to determine balance status")
        
    print("\n💡 Tip: Set your API key with: export OPENAI_API_KEY='your-api-key-here'")