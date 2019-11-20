def strip_leading_space(string: str) -> str:
    if string[0] == " ":
        return strip_leading_space(string[1:])
    else:
        return string