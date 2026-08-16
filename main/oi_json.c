#include "oi_json.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static void skip_ws(oi_json_reader_t *r)
{
    while (r->pos < r->len && isspace((unsigned char)r->buf[r->pos])) r->pos++;
}

// Advances r->pos past a JSON value (string/object/array/number/literal)
// starting at r->pos. Returns false on malformed/truncated input.
static bool skip_value(oi_json_reader_t *r)
{
    skip_ws(r);
    if (r->pos >= r->len) return false;
    char c = r->buf[r->pos];

    if (c == '"') {
        r->pos++;
        while (r->pos < r->len && r->buf[r->pos] != '"') {
            if (r->buf[r->pos] == '\\') r->pos++;
            r->pos++;
        }
        if (r->pos >= r->len) return false;
        r->pos++;  // closing quote
        return true;
    }
    if (c == '{' || c == '[') {
        char open = c, close = (c == '{') ? '}' : ']';
        int depth = 0;
        do {
            if (r->pos >= r->len) return false;
            char ch = r->buf[r->pos];
            if (ch == '"') {
                r->pos++;
                while (r->pos < r->len && r->buf[r->pos] != '"') {
                    if (r->buf[r->pos] == '\\') r->pos++;
                    r->pos++;
                }
                if (r->pos >= r->len) return false;
            } else if (ch == open) {
                depth++;
            } else if (ch == close) {
                depth--;
            }
            r->pos++;
        } while (depth > 0);
        return true;
    }
    // number / true / false / null
    while (r->pos < r->len &&
           r->buf[r->pos] != ',' && r->buf[r->pos] != '}' && r->buf[r->pos] != ']' &&
           !isspace((unsigned char)r->buf[r->pos])) {
        r->pos++;
    }
    return true;
}

void oi_json_reader_init(oi_json_reader_t *r, const char *buf, size_t len)
{
    r->buf = buf;
    r->len = len;
    r->pos = 0;
    skip_ws(r);
    if (r->pos < r->len && r->buf[r->pos] == '{') r->pos++;  // enter root object
}

bool oi_json_next_entry(oi_json_reader_t *r,
                         const char **key_out, size_t *key_len,
                         const char **obj_start, size_t *obj_len)
{
    skip_ws(r);
    if (r->pos >= r->len) return false;

    // Skip separators / end of object
    while (r->pos < r->len && (r->buf[r->pos] == ',' )) { r->pos++; skip_ws(r); }
    if (r->pos >= r->len || r->buf[r->pos] == '}') return false;

    if (r->buf[r->pos] != '"') return false;
    r->pos++;
    const char *kstart = r->buf + r->pos;
    while (r->pos < r->len && r->buf[r->pos] != '"') r->pos++;
    if (r->pos >= r->len) return false;
    *key_out = kstart;
    *key_len = (size_t)(r->buf + r->pos - kstart);
    r->pos++;  // closing quote

    skip_ws(r);
    if (r->pos >= r->len || r->buf[r->pos] != ':') return false;
    r->pos++;
    skip_ws(r);

    if (r->pos >= r->len || r->buf[r->pos] != '{') return false;
    const char *ostart = r->buf + r->pos;
    if (!skip_value(r)) return false;
    *obj_start = ostart;
    *obj_len   = (size_t)(r->buf + r->pos - ostart);
    return true;
}

// Finds `"field":` inside [obj, obj+obj_len) and positions *val_start at the
// first char of the value. Returns false if the field isn't present at the
// top level of this object (nested objects with the same field name are
// skipped, since skip_value consumes them whole).
static bool find_field(const char *obj, size_t obj_len, const char *field, size_t *val_pos)
{
    oi_json_reader_t r;
    // Treat [obj, obj_len) as a root object body so oi_json_next_entry can walk it.
    r.buf = obj;
    r.len = obj_len;
    r.pos = 0;
    skip_ws(&r);
    if (r.pos < r.len && r.buf[r.pos] == '{') r.pos++;

    const char *key; size_t klen;
    size_t field_len = strlen(field);
    while (true) {
        skip_ws(&r);
        if (r.pos >= r.len) return false;
        while (r.pos < r.len && r.buf[r.pos] == ',') { r.pos++; skip_ws(&r); }
        if (r.pos >= r.len || r.buf[r.pos] == '}') return false;
        if (r.buf[r.pos] != '"') return false;
        r.pos++;
        key = r.buf + r.pos;
        while (r.pos < r.len && r.buf[r.pos] != '"') r.pos++;
        if (r.pos >= r.len) return false;
        klen = (size_t)(r.buf + r.pos - key);
        r.pos++;
        skip_ws(&r);
        if (r.pos >= r.len || r.buf[r.pos] != ':') return false;
        r.pos++;
        skip_ws(&r);

        if (klen == field_len && memcmp(key, field, field_len) == 0) {
            *val_pos = r.pos;
            return true;
        }
        if (!skip_value(&r)) return false;
    }
}

bool oi_json_get_number(const char *obj, size_t obj_len, const char *field, double *out)
{
    size_t vpos;
    if (!find_field(obj, obj_len, field, &vpos)) return false;
    if (vpos >= obj_len) return false;
    char tmp[32];
    size_t n = 0;
    while (vpos + n < obj_len && n < sizeof(tmp) - 1 &&
           (isdigit((unsigned char)obj[vpos + n]) || obj[vpos + n] == '-' ||
            obj[vpos + n] == '+' || obj[vpos + n] == '.' ||
            obj[vpos + n] == 'e' || obj[vpos + n] == 'E')) {
        tmp[n] = obj[vpos + n];
        n++;
    }
    if (n == 0) return false;
    tmp[n] = '\0';
    *out = atof(tmp);
    return true;
}

bool oi_json_get_string(const char *obj, size_t obj_len, const char *field,
                         char *out, size_t out_size)
{
    size_t vpos;
    if (!find_field(obj, obj_len, field, &vpos)) return false;
    if (vpos >= obj_len || obj[vpos] != '"') return false;
    vpos++;
    size_t n = 0;
    while (vpos + n < obj_len && obj[vpos + n] != '"' && n < out_size - 1) {
        out[n] = obj[vpos + n];
        n++;
    }
    out[n] = '\0';
    return true;
}

bool oi_json_get_bool(const char *obj, size_t obj_len, const char *field, bool *out)
{
    size_t vpos;
    if (!find_field(obj, obj_len, field, &vpos)) return false;
    if (vpos + 4 <= obj_len && memcmp(obj + vpos, "true", 4) == 0) { *out = true; return true; }
    if (vpos + 5 <= obj_len && memcmp(obj + vpos, "false", 5) == 0) { *out = false; return true; }
    return false;
}
