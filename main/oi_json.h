#pragma once
// Minimal JSON reader used by oi_can.c to pull small integer/string/double
// fields out of the inverter's parameter-schema JSON. Not a general-purpose
// parser — just enough to walk a flat object-of-objects and read named
// number/string fields out of each child object, which is all the ported
// SDO client needs (see esp32-web-interface's PROTOCOL.md JSON Mapping).
#include <stddef.h>
#include <stdbool.h>

typedef struct {
    const char *buf;
    size_t      len;
    size_t      pos;
} oi_json_reader_t;

void oi_json_reader_init(oi_json_reader_t *r, const char *buf, size_t len);

// Advances to the next top-level "key": { ... } pair in the root object.
// On success, *key_out/*key_len is the key text (no quotes), and
// *obj_start/*obj_len bounds the child object's "{ ... }" text (inclusive).
// Returns false at end of object / on malformed input.
bool oi_json_next_entry(oi_json_reader_t *r,
                         const char **key_out, size_t *key_len,
                         const char **obj_start, size_t *obj_len);

// Look up a numeric field (e.g. "id", "i") within a child object's text
// (as returned by oi_json_next_entry). Returns false if absent/non-numeric.
bool oi_json_get_number(const char *obj, size_t obj_len, const char *field, double *out);

// Look up a string field's raw (unescaped) contents within a child object.
// Copies up to out_size-1 bytes into out, NUL-terminated. Returns false if absent.
bool oi_json_get_string(const char *obj, size_t obj_len, const char *field,
                         char *out, size_t out_size);

// Look up a boolean field ("true"/"false"). Returns false if absent.
bool oi_json_get_bool(const char *obj, size_t obj_len, const char *field, bool *out);
