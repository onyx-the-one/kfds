from docling.document_converter import DocumentConverter, PdfFormatOption
from docling.datamodel.base_models import InputFormat
from docling.datamodel.pipeline_options import PdfPipelineOptions, smolvlm_picture_description

# 1. Configure the pipeline to find images and generate descriptions
pipeline_options = PdfPipelineOptions()
pipeline_options.generate_picture_images = True
pipeline_options.do_picture_description = True

# 2. Attach the tiny local vision model (SmolVLM)
pipeline_options.picture_description_options = smolvlm_picture_description

# 3. Give it a custom prompt tailored to electrotechnical hardware
pipeline_options.picture_description_options.prompt = (
    "You are an electrical engineer. Describe this hardware diagram, schematic, "
    "or pinout in detail so another engineer can understand it from text alone. "
    "List visible pins, labels, and connections."
)

# 4. Load the converter with our options
converter = DocumentConverter(
    format_options={
        InputFormat.PDF: PdfFormatOption(pipeline_options=pipeline_options)
    }
)

print("Processing PDF and analyzing diagrams... (This may take a few minutes on CPU)")
doc = converter.convert("datasheet.pdf").document

# 5. Save the output to a Markdown file
with open("datasheet_context.md", "w") as f:
    f.write(doc.export_to_markdown())

print("Done! You can now /read datasheet_context.md in Aider.")
