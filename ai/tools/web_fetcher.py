import urllib.request
import urllib.parse
import json
import logging

logger = logging.getLogger("archer.web_fetcher")

def get_weather(location: str = "Nottingham, UK") -> str:
    """Fetches current weather for a location using wttr.in"""
    if not location or location.strip() == "":
        location = "Nottingham, UK"
    
    # encode the location for URL
    encoded_loc = urllib.parse.quote(location)
    url = f"https://wttr.in/{encoded_loc}?format=%l:+%C+%t"
    
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'curl/7.68.0'})
        with urllib.request.urlopen(req, timeout=5) as response:
            data = response.read().decode('utf-8').strip()
            return f"Weather report: {data}"
    except Exception as e:
        logger.error(f"Weather fetch failed: {e}")
        return "Weather data is currently unavailable."

def search_wikipedia(query: str) -> str:
    """Fetches a short summary from Wikipedia for the query"""
    if not query:
        return "Search query was empty."
        
    encoded_query = urllib.parse.quote(query)
    # First search for the most relevant page title
    search_url = f"https://en.wikipedia.org/w/api.php?action=query&list=search&srsearch={encoded_query}&utf8=&format=json&srlimit=1"
    
    try:
        req = urllib.request.Request(search_url, headers={'User-Agent': 'Archer-AI-Robot/1.0'})
        with urllib.request.urlopen(req, timeout=5) as response:
            search_data = json.loads(response.read().decode('utf-8'))
            search_results = search_data.get('query', {}).get('search', [])
            if not search_results:
                return f"No information found for '{query}'."
            
            title = search_results[0]['title']
            
        # Then get the summary for that page
        encoded_title = urllib.parse.quote(title)
        summary_url = f"https://en.wikipedia.org/api/rest_v1/page/summary/{encoded_title}"
        req2 = urllib.request.Request(summary_url, headers={'User-Agent': 'Archer-AI-Robot/1.0'})
        with urllib.request.urlopen(req2, timeout=5) as response2:
            summary_data = json.loads(response2.read().decode('utf-8'))
            extract = summary_data.get('extract', '')
            if extract:
                return f"Information about {title}: {extract}"
            return f"Could not extract summary for '{title}'."
            
    except Exception as e:
        logger.error(f"Wikipedia search failed: {e}")
        return "Information search is currently unavailable."
