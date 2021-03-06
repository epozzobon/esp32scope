#define STRLEN(s) (sizeof(s)/sizeof(s[0])-sizeof(s[0]))

const char resp_notfound_hdr[] =
    "HTTP/1.0 404 NOT FOUND\r\n"
    "Content-Type: text/html\r\n"
    "\r\n";
size_t resp_notfound_hdr_len = STRLEN(resp_notfound_hdr);

const char resp_error_hdr[] =
    "HTTP/1.0 500 INTERNAL SERVER ERROR\r\n"
    "Content-Type: text/html\r\n"
    "\r\n";
size_t resp_error_hdr_len = STRLEN(resp_error_hdr);


const char resp_binary_hdr[] = 
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: application/octet-stream\r\n"
    "\r\n";
size_t resp_binary_hdr_len = STRLEN(resp_binary_hdr);


const char resp_html_hdr[] =
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "\r\n";
size_t resp_html_hdr_len = STRLEN(resp_html_hdr);

const char resp_icon_hdr[] =
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: image/x-icon\r\n"
    "\r\n";
size_t resp_icon_hdr_len = STRLEN(resp_icon_hdr);


extern const uint8_t webpage_index_start[] asm("_binary_index_html_start");
extern const uint8_t webpage_index_end[]   asm("_binary_index_html_end");
#define webpage_index_length (webpage_index_end - webpage_index_start - 1)

extern const uint8_t webpage_notfound_start[] asm("_binary_notfound_html_start");
extern const uint8_t webpage_notfound_end[]   asm("_binary_notfound_html_end");
#define webpage_notfound_length (webpage_notfound_end - webpage_notfound_start - 1)

extern const uint8_t webpage_error_start[] asm("_binary_error_html_start");
extern const uint8_t webpage_error_end[]   asm("_binary_error_html_end");
#define webpage_error_length (webpage_error_end - webpage_error_start - 1)

extern const uint8_t webpage_dygraph_js_start[] asm("_binary_dygraph_js_start");
extern const uint8_t webpage_dygraph_js_end[]   asm("_binary_dygraph_js_end");
#define webpage_dygraph_js_length (webpage_dygraph_js_end - webpage_dygraph_js_start - 1)

extern const uint8_t webpage_dygraph_css_start[] asm("_binary_dygraph_css_start");
extern const uint8_t webpage_dygraph_css_end[]   asm("_binary_dygraph_css_end");
#define webpage_dygraph_css_length (webpage_dygraph_css_end - webpage_dygraph_css_start - 1)

extern const uint8_t webpage_favicon_ico_start[] asm("_binary_favicon_ico_start");
extern const uint8_t webpage_favicon_ico_end[]   asm("_binary_favicon_ico_end");
#define webpage_favicon_ico_length (webpage_favicon_ico_end - webpage_favicon_ico_start - 1)