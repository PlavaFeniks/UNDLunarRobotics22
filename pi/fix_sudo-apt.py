import pathlib


def main() -> None:
    sources = []
    sources_path = pathlib.Path("/etc/apt/sources.list")
    sources.append(sources_path)
    sources.append(sources_path.parent / "sources.list.d/raspi.list")
    
    for apt_source in sources_path:
        changeFiles(apt_source)

def changeFiles(source_path: pathlib.Path) -> None:
    text_to_replace = "#deb-src"
    with open(source_path, "wr") as f:
        text = f.read()
        text = text.replace(text_to_replace, text_to_replace[1::])
        f.write(text)
        f.close()
    
    print(f"Succesfully updated {source_path}")



if __name__ == "__main__":
        main()